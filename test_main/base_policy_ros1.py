#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging
import numpy as np
import threading
import cv2
import rospy
from dataclasses import dataclass
from typing import Optional
from message_filters import Subscriber as MFSubscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import CompressedImage, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

@dataclass
class Frame:
    rgb_image: np.ndarray
    depth_image: np.ndarray
    current_state: np.ndarray
    current_depth_state: np.ndarray
    stamp: float

class BasePolicyNode(object):
    def __init__(self, node_name: str = "base_policy_node"):
        # 不在此 init_node；由外部 main 初始化 rospy
        self.frame_lock = threading.Lock()
        self.frame: Optional[Frame] = None
        self.depth_info = None
        self.extrinsics = np.array([0.15, 0, 0.1], dtype=np.float64)
        self.safe_dis = 0.4

        # message_filters 同步订阅
        self.odom_sub  = MFSubscriber('/unity_depth_odom1', Odometry, queue_size=10)
        self.rgb_sub   = MFSubscriber('/camera1/color/image/compressed', CompressedImage, queue_size=10)
        self.depth_sub = MFSubscriber('/camera1/depth/image/compressed', CompressedImage, queue_size=10)

        self.ts = ApproximateTimeSynchronizer(
            [self.odom_sub, self.rgb_sub, self.depth_sub],
            queue_size=45, slop=0.1, allow_headerless=False
        )
        self.ts.registerCallback(self.synced_callback)
        rospy.loginfo('message_filters 同步器已启动')

        # 额外订阅一次性相机内参（ROS1 用 rospy.Subscriber）
        self.depth_info_sub = rospy.Subscriber(
            'camera1/depth/info', CameraInfo, self.depth_info_callback, queue_size=1
        )

        # 发布当前机器人状态
        self.state_pub = rospy.Publisher("robot/current_state", PoseStamped, queue_size=10)

    def get_frame_snapshot(self) -> Optional[Frame]:
        with self.frame_lock:
            return self.frame

    def synced_callback(self, odom_msg: Odometry,
                        rgb_msg: CompressedImage,
                        depth_msg: CompressedImage):
        try:
            # 1) 读取（相机=里程计坐标）世界位姿
            pos = odom_msg.pose.pose.position
            ori = odom_msg.pose.pose.orientation
            roll_c, pitch_c, yaw_c = self.quaternion_to_euler(ori.x, ori.y, ori.z, ori.w)

            R_wr   = self.euler_rpy_to_R(roll_c, pitch_c, yaw_c)   # 机器人(=相机)->世界
            p_w_cam = np.array([pos.x, pos.y, pos.z], dtype=np.float64)

            # p_w_cam = R_wr * t_rc + p_w_robot  =>  p_w_robot = p_w_cam - R_wr * t_rc
            p_w_robot = p_w_cam - (R_wr @ self.extrinsics)

            current_depth_state = np.array([p_w_cam[0], p_w_cam[1], p_w_cam[2],
                                            roll_c, pitch_c, yaw_c], dtype=np.float64)
            current_state = np.array([p_w_robot[0], p_w_robot[1], p_w_robot[2],
                                      roll_c, pitch_c, yaw_c], dtype=np.float64)

            # 2) 解码图像
            np_arr = np.frombuffer(rgb_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            rgb_image = cv_image.copy()

            np_arr = np.frombuffer(depth_msg.data, np.uint8)
            depth = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

            # 假设深度编码：8bit 0..255 -> 0..10 m
            if depth.ndim == 3:
                depth_m = depth[:, :, 0]
            else:
                depth_m = depth
            depth_32 = depth_m.astype(np.float32) / 255.0 * 10.0
            depth_32[depth_32 == 0] = 5.1
            depth_image = depth_32.copy()

            stamp = odom_msg.header.stamp.to_sec()

            frame = Frame(rgb_image=rgb_image,
                          depth_image=depth_image,
                          current_state=current_state,
                          current_depth_state=current_depth_state,
                          stamp=stamp)
            with self.frame_lock:
                self.publish_current_state(current_state, stamp)
                self.frame = frame

        except Exception as e:
            logging.error(f"同步回调处理错误: {e}")

    def depth_info_callback(self, msg: CameraInfo):
        """处理深度相机参数回调（只取一次后注销订阅）"""
        if self.depth_info is None:
            self.depth_info = {
                'height': msg.height, 'width': msg.width,
                'fx': msg.K[0], 'fy': msg.K[4],
                'cx': msg.K[2], 'cy': msg.K[5],
                'fov': 57.0
            }
            try:
                self.depth_info_sub.unregister()
            except Exception:
                pass

    def quaternion_to_euler(self, x, y, z, w):
        """四元数 -> 欧拉角 (roll, pitch, yaw)"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return roll, pitch, yaw

    def euler_to_quaternion(self, roll, pitch, yaw):
        """欧拉角 -> 四元数 (x, y, z, w)"""
        cy = np.cos(yaw * 0.5); sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5); sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5);  sr = np.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def euler_rpy_to_R(self, roll, pitch, yaw):
        """R = Rz(yaw) @ Ry(pitch) @ Rx(roll)"""
        sr, cr = np.sin(roll),  np.cos(roll)
        sp, cp = np.sin(pitch), np.cos(pitch)
        sy, cy = np.sin(yaw),   np.cos(yaw)
        Rx = np.array([[1, 0, 0],
                       [0, cr, -sr],
                       [0, sr,  cr]], dtype=np.float64)
        Ry = np.array([[ cp, 0, sp],
                       [  0, 1,  0],
                       [-sp, 0, cp]], dtype=np.float64)
        Rz = np.array([[cy, -sy, 0],
                       [sy,  cy, 0],
                       [ 0,   0, 1]], dtype=np.float64)
        return Rz @ Ry @ Rx

    def _backproject(self, u, v, depth_value, fx, fy, cx, cy, mode="z"):
        d = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=np.float64)
        if mode == "z":
            Z = depth_value
            return d * Z
        elif mode == "range":
            s = d / np.linalg.norm(d)
            return s * depth_value
        else:
            raise ValueError(f"Unknown depth mode: {mode}")

    def bbox_center(self, results):
        if not results:
            raise ValueError("Results list is empty")
        bbox = results[0].get("bbox_2d", None)
        if bbox is None or len(bbox) != 4:
            raise ValueError("Invalid bbox_2d format")
        x1, y1, x2, y2 = bbox
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        return center_x, center_y

    def _to_rad(self, ang):
        """度/弧度自适应：若|ang|>pi视为度并转弧度"""
        if isinstance(ang, (list, tuple, np.ndarray)):
            return [self._to_rad(a) for a in ang]
        return np.deg2rad(ang) if abs(float(ang)) > np.pi else float(ang)

    def _fov_hv_from_info(self, fov_info, W, H):
        """
        接受：
        - 标量：默认为水平FOV
        - 字典：{'h':.., 'v':..} / 仅 'h' / 仅 'v' / {'diag':..}
        返回：fh, fv（弧度）
        """
        r = W / float(H)
        if isinstance(fov_info, dict):
            if 'h' in fov_info and 'v' in fov_info:
                fh, fv = self._to_rad(fov_info['h']), self._to_rad(fov_info['v'])
            elif 'h' in fov_info:
                fh = self._to_rad(fov_info['h'])
                fv = 2.0 * np.arctan(np.tan(fh/2.0) * (1.0/r))
            elif 'v' in fov_info:
                fv = self._to_rad(fov_info['v'])
                fh = 2.0 * np.arctan(np.tan(fv/2.0) * r)
            elif 'diag' in fov_info:
                fd = self._to_rad(fov_info['diag'])
                a = np.tan(fd/2.0)
                t = a / np.sqrt(r*r + 1.0)
                fv = 2.0 * np.arctan(t)
                fh = 2.0 * np.arctan(r * t)
            else:
                raise ValueError("fov dict 需含 'h'/'v'/'diag' 之一")
        else:
            fh = self._to_rad(fov_info)
            fv = 2.0 * np.arctan(np.tan(fh/2.0) * (1.0/r))
        return fh, fv

    def _backproject_from_fov(self, u, v, depth_value, W, H, fov_info, mode="z"):
        """用FOV回投到相机光学系(OpenCV: x右、y下、z前)"""
        fh, fv = self._fov_hv_from_info(fov_info, W, H)
        cx = (W - 1) * 0.5
        cy = (H - 1) * 0.5
        nx = (u - cx) / max(cx, 1e-9)
        ny = (v - cy) / max(cy, 1e-9)
        thx = nx * (fh * 0.5)
        thy = ny * (fv * 0.5)
        d = np.array([np.tan(thx), np.tan(thy), 1.0], dtype=np.float64)
        if mode == "z":
            return d * float(depth_value)
        elif mode == "range":
            s = d / np.linalg.norm(d)
            return s * float(depth_value)
        else:
            raise ValueError(f"Unknown depth mode: {mode}")

    def pixel_to_world(self, results, frame: Frame,
                       depth_scale=1.0, window=0, depth_mode="z"):
        """
        输入：检测结果(像素 u,v)、帧快照
        输出：像素在世界系中的三维坐标 P_w 及是否需要重规划 replan
        """
        if self.depth_info is None:
            raise ValueError("相机参数未就绪")

        replan = False

        # ---------- 0) 读入像素与相机参数 ----------
        u, v = float(results[0]), float(results[1])
        fx, fy, cx, cy = (self.depth_info['fx'], self.depth_info['fy'],
                          self.depth_info['cx'], self.depth_info['cy'])

        H, W = frame.depth_image.shape[:2]
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < W and 0 <= vi < H):
            raise ValueError(f"(u,v)=({ui},{vi}) 超出图像范围 {W}x{H}")

        # ---------- 1) 获取深度 ----------
        if window > 0:
            u0, u1 = max(0, ui - window), min(W - 1, ui + window)
            v0, v1 = max(0, vi - window), min(H - 1, vi + window)
            patch = frame.depth_image[v0:v1+1, u0:u1+1].astype(np.float64)
            depth_raw = np.median(patch)
        else:
            depth_raw = float(frame.depth_image[vi, ui])

        if depth_raw > 5.0 + 1e-6:  # 你的哨兵值逻辑
            replan = True
            depth_raw = 5.0

        depth_val = depth_raw * depth_scale
        if not np.isfinite(depth_val) or depth_val <= 0:
            raise ValueError(f"无效深度 depth={depth_val}")

        print(f"深度为:{depth_val}")

        # ---------- 2) 像素→相机光学系 ----------
        fov_info = self.depth_info['fov']
        P_opt = self._backproject_from_fov(u, v, depth_val, W, H, fov_info, depth_mode)

        # ---------- 3) 光学系→相机机体系（x前、y左、z上） ----------
        R_opt2cam = np.array([
            [ 0,  0,  1],   # x_cam =  z_opt
            [-1,  0,  0],   # y_cam = -x_opt
            [ 0, -1,  0],   # z_cam = -y_opt
        ], dtype=np.float64)
        P_cam = R_opt2cam @ P_opt

        # ---------- 4) 相机机体系→世界系（使用深度相机世界位姿） ----------
        tx_c, ty_c, tz_c, roll_c, pitch_c, yaw_c = map(float, frame.current_depth_state.tolist())
        R_w_cam = self.euler_rpy_to_R(roll_c, pitch_c, yaw_c)
        t_wc = np.array([tx_c, ty_c, tz_c], dtype=np.float64)

        # ---------- 5) 安全距离 ----------
        dist_cam = float(np.linalg.norm(P_cam))
        if dist_cam <= self.safe_dis + 1e-6:
            P_cam_safe = np.zeros(3, dtype=np.float64)
            replan = True or replan
        else:
            P_cam_safe = P_cam * ((dist_cam - self.safe_dis) / dist_cam)

        # ---------- 6) 到世界系 ----------
        P_w = R_w_cam @ P_cam_safe + t_wc
        return P_w, replan

    def publish_current_state(self, current_state: np.ndarray, stamp: float):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.from_sec(stamp)
        msg.header.frame_id = "world"
        msg.pose.position.x = float(current_state[0])
        msg.pose.position.y = float(current_state[1])
        msg.pose.position.z = float(current_state[2])
        qx, qy, qz, qw = self.euler_to_quaternion(
            current_state[3], current_state[4], current_state[5]
        )
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.state_pub.publish(msg)

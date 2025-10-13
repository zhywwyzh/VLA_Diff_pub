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
import pdb
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
import math
import pdb


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
        self.safe_dis = 0.6
        self.use_intrinsics = True
        self.depth_scale_param = 57.0

        # message_filters 同步订阅
        self.odom_sub  = MFSubscriber('/unity_odom', Odometry, queue_size=10)
        self.rgb_sub   = MFSubscriber('/camera1/color/image/compressed', CompressedImage, queue_size=10)
        self.depth_sub = MFSubscriber('/camera0/depth/image/compressed', CompressedImage, queue_size=10)

        self.pc_frame_id   = rospy.get_param("~pc_frame_id", "world")  # 点云坐标系，默认世界系
        self.pc_stride     = int(rospy.get_param("~pc_stride", 1))     # 采样步长，=1表示逐像素
        self.pc_max_range  = float(rospy.get_param("~pc_max_range", 30.0))  # 超过此距离的深度视为无效
        self.pc_keep_far   = bool(rospy.get_param("~pc_keep_far", False))    # 是否保留“最远平面”(30m)的点

        # 点云发布器 + 定时器（10Hz）
        self.cloud_pub = rospy.Publisher("cloud_depth", PointCloud2, queue_size=1)
        self.cloud_timer = rospy.Timer(rospy.Duration(0.1), self._cloud_timer_cb)

        self.ts = ApproximateTimeSynchronizer(
            [self.odom_sub, self.rgb_sub, self.depth_sub],
            queue_size=45, slop=0.1, allow_headerless=False
        )
        self.ts.registerCallback(self.synced_callback)
        rospy.loginfo('message_filters 同步器已启动')

        # 额外订阅一次性相机内参（ROS1 用 rospy.Subscriber）
        self.depth_info_sub = rospy.Subscriber(
            'camera1/camera_info', CameraInfo, self.depth_info_callback, queue_size=1
        )

        # 发布当前机器人状态
        self.state_pub = rospy.Publisher("robot/current_state", PoseStamped, queue_size=10)

        # 发布目标点marker
        self.marker_pub = rospy.Publisher("pixel_to_world_marker", Marker, queue_size=10)

    def get_frame_snapshot(self) -> Optional[Frame]:
        with self.frame_lock:
            return self.frame

    def synced_callback(self, odom_msg: Odometry,
                        rgb_msg: CompressedImage,
                        depth_msg: CompressedImage):
        # 1) 解码图像
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

        depth_32 = (255.0 - depth_m.astype(np.float32)) / 255.0 * 5.0

        # print(f"depth信息：{depth_32}")
        depth_32[depth_32 < 1e-6] = 5.1
        depth_image = depth_32.copy()
        # 2) 读取（相机=里程计坐标）世界位姿
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


        stamp = odom_msg.header.stamp.to_sec()

        frame = Frame(rgb_image=rgb_image,
                        depth_image=depth_image,
                        current_state=current_state,
                        current_depth_state=current_depth_state,
                        stamp=stamp)
        with self.frame_lock:
            self.publish_current_state(current_state, stamp)
            self.frame = frame

    def depth_info_callback(self, msg: CameraInfo):
        """处理深度相机参数回调（只取一次后注销订阅）"""
        if self.depth_info is None:
            self.depth_info = {
                'height': msg.height, 'width': msg.width,
                'fx': msg.K[0], 'fy': msg.K[4],
                'cx': msg.K[2], 'cy': msg.K[5],
                'fov': 57.0
            }
            self.depth_info_sub.unregister()

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
        roll, pitch, yaw = map(float, frame.current_state[3:6].tolist())
        yaw = yaw - results.get("yaw", 0.0) / 180.0 * math.pi
        if results["pos"] == [-1, -1]:
            P_w = [frame.current_state[0], frame.current_state[1], frame.current_state[2]]
            P_w_full = np.concatenate([P_w, np.array([roll, pitch, yaw], dtype=np.float64)])
            replan = False
            return P_w_full, replan

        # ---------- 0) 读入像素与相机参数 ----------
        u, v = float(results["pos"][0]), float(results["pos"][1])
        fx, fy, cx, cy = (self.depth_info['fx'], self.depth_info['fy'],
                          self.depth_info['cx'], self.depth_info['cy'])

        # print(f"fx:{fx}, fy:{fy}, cx:{cx}, cy:{cy}")

        # pdb.set_trace()
        # 方向（OpenCV 光学系：x右、y下、z前）
        dx = (u - cx) / (fx if abs(fx) > 1e-12 else 1e-12)
        dy = (v - cy) / (fy if abs(fy) > 1e-12 else 1e-12)
        dz = np.ones_like(dx)

        H, W = frame.depth_image.shape[:2]
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < W and 0 <= vi < H):
            raise ValueError(f"(u,v)=({ui},{vi}) 超出图像范围 {W}x{H}")
        # pdb.set_trace()

        # ---------- 1) 获取深度 ----------
        if window > 0:
            u0, u1 = max(0, ui - window), min(W - 1, ui + window)
            v0, v1 = max(0, vi - window), min(H - 1, vi + window)
            patch = frame.depth_image[v0:v1+1, u0:u1+1].astype(np.float64)
            depth_raw = np.median(patch)
        else:
            depth_raw = float(frame.depth_image[vi, ui])

        if depth_raw > 5.0 + 1e-6:  # 哨兵值逻辑
            replan = True
            depth_raw = 5.0
        # print(f"depth:{depth_raw}")
        if depth_raw > 1.0 + 1e-6:
            depth_val = (depth_raw - 0.6) * depth_scale
        else:
            depth_val = depth_raw * depth_scale
        if not np.isfinite(depth_val) or depth_val <= 0:
            raise ValueError(f"无效深度 depth={depth_val}")

        # print(f"深度为:{depth_val}")

        # ---------- 2) 像素→相机光学系 ----------
        if self.use_intrinsics:
            # P_opt = self._backproject(u, v, depth_val, fx, fy, cx, cy, mode=depth_mode)
            P_opt = np.array([dx * depth_val, dy * depth_val, dz * depth_val], dtype=np.float64)
        else:
            fov_info = self.depth_info['fov']
            P_opt = self._backproject_from_fov(u, v, depth_val, W, H, fov_info, mode=depth_mode)
        # ---------- 3) 光学系→相机机体系（x前、y左、z上） ----------
        R_opt2cam = np.array([
            [ 0,  0,  1],   # x_cam =  z_opt
            [-1,  0,  0],   # y_cam = -x_opt
            [ 0, -1,  0],   # z_cam = -y_opt
        ], dtype=np.float64)
        # P_cam = R_opt2cam @ P_opt
        P_cam = (R_opt2cam @ P_opt.T).T

        # ---------- 4) 相机机体系→世界系（使用深度相机世界位姿） ----------
        tx_c, ty_c, tz_c, roll_c, pitch_c, yaw_c = map(float, frame.current_depth_state.tolist())

        R_w_cam = self.euler_rpy_to_R(roll_c, pitch_c, yaw_c)
        t_wc = np.array([tx_c, ty_c, tz_c], dtype=np.float64)

        # # ---------- 5) 安全距离 ----------
        # dist_cam = float(np.linalg.norm(P_cam))
        # # if np.isfinite(dist_cam) and dist_cam < self.safe_dis:
        # rospy.loginfo(f"相机距离安全边界: {dist_cam:.2f}")
        # if dist_cam > 1e-6:
        #     P_cam_safe = P_cam * (self.safe_dis / dist_cam)
        # else:
        #     # 万一刚好在相机中心，给一个前方安全半径的默认方向
        #     P_cam_safe = np.array([self.safe_dis, 0.0, 0.0], dtype=np.float64)
        # replan = True or replan
        # else:
        #     P_cam_safe = P_cam

        # ---------- 6) 到世界系 ----------
        # P_w = R_w_cam @ P_cam_safe + t_wc
        P_w = (R_w_cam @ P_cam.T).T + t_wc  # [N,3]
        P_w_full = np.concatenate([P_w, np.array([roll, pitch, yaw], dtype=np.float64)])
        self._publish_p2w_marker(P_w_full, frame.stamp)
        return P_w_full, replan

    def calculate_posture(self, current_state, delta_yaw):
        """计算姿态"""
        if delta_yaw > 0.0 + 1e-6:
            current_state[5] += delta_yaw
        return current_state

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

    def _cloud_timer_cb(self, event):
        """10Hz 定时发布深度点云"""
        if self.depth_info is None:
            return
        frame = self.get_frame_snapshot()
        if frame is None or frame.depth_image is None:
            return
        cloud_msg = self._build_pointcloud_from_frame(frame)
        if cloud_msg is not None:
            self.cloud_pub.publish(cloud_msg)

    def _build_pointcloud_from_frame(self, frame: Frame) -> Optional[PointCloud2]:
        """
        使用与 pixel_to_world 相同的回投与位姿变换逻辑，遍历整幅深度图生成世界系点云（或你设定的frame_id）
        """
        depth = frame.depth_image
        H, W = depth.shape[:2]
        if H == 0 or W == 0:
            return None

        # 生成像素网格（可通过 stride 下采样）
        s = max(1, int(self.pc_stride))
        us = np.arange(0, W, s, dtype=np.float64)
        vs = np.arange(0, H, s, dtype=np.float64)
        uu, vv = np.meshgrid(us, vs)     # 形状: [H/s, W/s]

        # 对应的深度取值
        depth_vals = depth[vv.astype(np.int64), uu.astype(np.int64)].astype(np.float64)

        # 过滤无效/哨兵深度：> 30 视为无效（或按需保留为 30.0）
        if self.pc_keep_far:
            # 保留远处为最大距离：将 > max_range 的值钳制为 max_range
            depth_vals = np.where(depth_vals > self.pc_max_range, self.pc_max_range, depth_vals)
            valid = np.isfinite(depth_vals) & (depth_vals > 0.0)
        else:
            valid = np.isfinite(depth_vals) & (depth_vals > 0.0) & (depth_vals <= self.pc_max_range)

        if not np.any(valid):
            return None

        uu = uu[valid]
        vv = vv[valid]
        depth_vals = depth_vals[valid]

        depth_mode = self._backproject  # "z" 或 "range"

        if self.use_intrinsics:
            # ---------- 内参回投 ----------
            fx = float(self.depth_info['fx']); fy = float(self.depth_info['fy'])
            cx = float(self.depth_info['cx']); cy = float(self.depth_info['cy'])

            # 方向（OpenCV 光学系：x右、y下、z前）
            dx = (uu - cx) / (fx if abs(fx) > 1e-12 else 1e-12)
            dy = (vv - cy) / (fy if abs(fy) > 1e-12 else 1e-12)
            dz = np.ones_like(dx)

            if depth_mode == "z":
                # Z 为光轴深度
                P_opt = np.stack([dx * depth_vals, dy * depth_vals, dz * depth_vals], axis=1)  # [N,3]
            else:  # "range"
                norm = np.sqrt(dx*dx + dy*dy + dz*dz) + 1e-12
                sx, sy, sz = dx / norm, dy / norm, dz / norm
                P_opt = np.stack([sx * depth_vals, sy * depth_vals, sz * depth_vals], axis=1)

        else:
            # ---------- FOV 近似回投（原逻辑保留） ----------
            fh, fv = self._fov_hv_from_info(self.depth_info['fov'], W, H)
            cx = (W - 1) * 0.5
            cy = (H - 1) * 0.5
            nx = (uu - cx) / max(cx, 1e-9)
            ny = (vv - cy) / max(cy, 1e-9)
            thx = nx * (fh * 0.5)
            thy = ny * (fv * 0.5)
            tx = np.tan(thx)
            ty = np.tan(thy)
            ones = np.ones_like(tx)
            if depth_mode == "z":
                P_opt = np.stack([tx * depth_vals, ty * depth_vals, ones * depth_vals], axis=1)
            else:  # "range"
                norm = np.sqrt(tx*tx + ty*ty + 1.0)
                sx, sy, sz = tx / norm, ty / norm, 1.0 / norm
                P_opt = np.stack([sx * depth_vals, sy * depth_vals, sz * depth_vals], axis=1)

        # 光学系 -> 相机机体系（x前、y左、z上），与 pixel_to_world 中一致
        R_opt2cam = np.array([
            [ 0,  0, 1],
            [-1,  0,  0],
            [ 0, -1,  0],
        ], dtype=np.float64)
        P_cam = (R_opt2cam @ P_opt.T).T  # [N,3]

        # 相机机体系 -> 目标坐标系（默认世界系）
        tx_c, ty_c, tz_c, roll_c, pitch_c, yaw_c = map(float, frame.current_depth_state.tolist())
        R_w_cam = self.euler_rpy_to_R(roll_c, pitch_c, yaw_c)
        t_wc = np.array([tx_c, ty_c, tz_c], dtype=np.float64)
        P_out = (R_w_cam @ P_cam.T).T + t_wc  # [N,3]

        # 生成 PointCloud2（xyz32）
        header = rospy.Header()
        header.stamp = rospy.Time.from_sec(frame.stamp)
        header.frame_id = self.pc_frame_id

        # 转 float32 以符合 xyz32
        points = P_out.astype(np.float32)
        cloud_msg = pc2.create_cloud_xyz32(header, points.tolist())
        return cloud_msg

    def _publish_p2w_marker(self, P_w, stamp):
        m = Marker()
        m.header.stamp = rospy.Time.from_sec(stamp)
        m.header.frame_id = self.pc_frame_id  # 通常为 "world"
        m.ns = "pixel_to_world"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(P_w[0])
        m.pose.position.y = float(P_w[1])
        m.pose.position.z = float(P_w[2])
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.3  # 直径 0.1 m
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        m.lifetime = rospy.Duration(0)  # 保持显示
        self.marker_pub.publish(m)

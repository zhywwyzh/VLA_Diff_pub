#!/usr/bin/env python3
import logging
import numpy as np
from rclpy.node import Node
from dataclasses import dataclass
from typing import Optional
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import CompressedImage, CameraInfo
from nav_msgs.msg import Odometry
import threading
import cv2
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped

@dataclass
class Frame:
    rgb_image: np.ndarray
    depth_image: np.ndarray
    current_state: np.ndarray
    current_depth_state: np.ndarray
    stamp: float

class BasePolicyNode(Node):
    def __init__(self, node_name: str = "base_policy_node"):
        super().__init__(node_name)
        self.frame_lock = threading.Lock()
        self.frame: Optional[Frame] = None
        self.depth_info = None
        self.extrinsics = [0.15, 0, 0.1]
        self.safe_dis = 0.4
        # 订阅无人机状态和图像话题
        self.odom_sub = Subscriber(self, Odometry, 
                                   '/unity_depth_odom1',
                                   qos_profile=100)

        self.rgb_sub = Subscriber(self, CompressedImage,
                                  '/camera1/color/image/compressed',
                                  qos_profile=10)

        self.depth_sub = Subscriber(self, CompressedImage,
                                    '/camera1/depth/image/compressed',
                                    qos_profile=10)
        
        # 发布参考位置
        self.state_pub = self.create_publisher(PoseStamped, "robot/current_state", 10)

        self.ts = ApproximateTimeSynchronizer(
            [self.odom_sub, self.rgb_sub, self.depth_sub],
            queue_size=45,
            slop=0.1,
            allow_headerless=False
        )
        self.ts.registerCallback(self.synced_callback)
        self.get_logger().info('message_filters 同步器已启动')

        # 订阅相机参数，仅订阅一次
        self.depth_info_sub = self.create_subscription(CameraInfo,
                                                       'camera1/depth/info',
                                                       self.depth_info_callback,
                                                       10)

    def get_frame_snapshot(self) -> Optional[Frame]:
        with self.frame_lock:
            return self.frame

    def synced_callback(self, odom_msg: Odometry,
                        rgb_msg: CompressedImage,
                        depth_msg: CompressedImage):
        try:
            # 1) 从里程计读取“相机”的世界位姿
            pos = odom_msg.pose.pose.position
            ori = odom_msg.pose.pose.orientation
            roll_c, pitch_c, yaw_c = self.quaternion_to_euler(ori.x, ori.y, ori.z, ori.w)

            # 假设相机与机器人在安装上无旋转差（若有，可在此加入固定R_rc旋转）
            R_wr = self.euler_rpy_to_R(roll_c, pitch_c, yaw_c)  # 机器人(=相机) → 世界
            p_w_cam = np.array([pos.x, pos.y, pos.z], dtype=np.float64)

            # 2) 用旋转矩阵把 extrinsics 从机器人系带到世界系，再相减得到机器人位姿
            # p_w_cam = R_wr * t_rc + p_w_robot  =>  p_w_robot = p_w_cam - R_wr * t_rc
            p_w_robot = p_w_cam - (R_wr @ self.extrinsics)

            # 相机位姿
            current_depth_state = np.array([p_w_cam[0], p_w_cam[1], p_w_cam[2],
                                             roll_c, pitch_c, yaw_c], dtype=np.float64)

            # 机器人姿态（若相机与机器人无旋转差，则相同）
            current_state = np.array([p_w_robot[0], p_w_robot[1], p_w_robot[2],
                                      roll_c, pitch_c, yaw_c], dtype=np.float64)

            # 3) 解码图像
            np_arr = np.frombuffer(rgb_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            rgb_image = cv_image.copy()

            np_arr = np.frombuffer(depth_msg.data, np.uint8)
            depth = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
 
            # 假设深度编码：8bit 0..255 -> 0..5 m
            depth_m = depth[:, :, 0]
            depth_32 = depth_m.astype(np.float32) / 255.0 * 5.0
            depth_32[depth_32 == 0] = 5.1
            depth_image = depth_32.copy()

            stamp = Time.from_msg(odom_msg.header.stamp).nanoseconds / 1e9

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

    def depth_info_callback(self, msg):
        """处理深度相机参数回调"""
        if self.depth_info is None:
            self.depth_info = {
                'height': msg.height, 'width': msg.width,
                'fx': msg.k[0], 'fy': msg.k[4],
                'cx': msg.k[2], 'cy': msg.k[5]
            }
            self.destroy_subscription(self.depth_info_sub)

    def quaternion_to_euler(self, x, y, z, w):
        """将四元数转换为欧拉角 (roll, pitch, yaw)"""
        # 这里实现了四元数到欧拉角的转换
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
        """将欧拉角转换为四元数"""
        # 这里实现了欧拉角到四元数的转换
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
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
        if mode == "z":  # projective depth
            Z = depth_value
            return d * Z
        elif mode == "range":  # ray length
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

    def pixel_to_world(self, results, frame: Frame,
                    depth_scale=1.0, window=0, depth_mode="z"):
        """
        输入：检测结果(含像素 u,v)、帧快照(含depth/rgb/相机/机器人位姿)
        输出：像素在世界系中的三维坐标 P_w（沿视线方向预留 self.safe_dis 安全距离）
        说明：深度图对应的相机世界位姿为 frame.current_depth_state；
            机器人世界位姿为 frame.current_state（此处不再用于坐标变换）。
        """
        if self.depth_info is None:
            raise ValueError("相机参数未就绪")

        replan = False

        # ---------- 0) 读入像素与相机内参 ----------
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

        if depth_raw == 5.1:       # 你的特殊哨兵值逻辑
            replan = True
            depth_raw = 5.0

        depth_val = depth_raw * depth_scale
        if not np.isfinite(depth_val) or depth_val <= 0:
            raise ValueError(f"无效深度 depth={depth_val}")

        # ---------- 2) 像素→相机光学系（OpenCV: x右、y下、z前） ----------
        P_opt = self._backproject(u, v, depth_val, fx, fy, cx, cy, depth_mode)

        # ---------- 3) 光学系→相机机体系（x前、y左、z上） ----------
        # 与此前一致：R_opt2cam 把 (右,下,前) 映射为 (前,左,上)
        R_opt2cam = np.array([
            [ 0,  0,  1],   # x_cam =  z_opt
            [-1,  0,  0],   # y_cam = -x_opt
            [ 0, -1,  0],   # z_cam = -y_opt
        ], dtype=np.float64)
        P_cam = R_opt2cam @ P_opt

        # ---------- 4) 从相机机体系→世界系：使用深度相机的世界位姿 ----------
        # frame.current_depth_state = [tx, ty, tz, roll, pitch, yaw]（相机机体系的RPY）
        tx_c, ty_c, tz_c, roll_c, pitch_c, yaw_c = map(float, frame.current_depth_state.tolist())
        R_w_cam = self.euler_rpy_to_R(roll_c, pitch_c, yaw_c)   # 相机机体系→世界
        t_wc = np.array([tx_c, ty_c, tz_c], dtype=np.float64)

        # ---------- 5) 安全距离处理：沿视线方向后退 self.safe_dis ----------
        # 在相机机体系中缩放到 (||P_cam|| - safe_dis)
        dist_cam = float(np.linalg.norm(P_cam))
        if dist_cam <= self.safe_dis + 1e-6:
            # 目标距离过近：保持在相机当前位置（或你可按需改为在相机前方极小偏移）
            P_cam_safe = np.zeros(3, dtype=np.float64)
            replan = True or replan
        else:
            P_cam_safe = P_cam * ((dist_cam - self.safe_dis) / dist_cam)

        # ---------- 6) 变换到世界系 ----------
        P_w = R_w_cam @ P_cam_safe + t_wc

        return P_w, replan


    def publish_current_state(self, current_state: np.ndarray, stamp: float):
        msg = PoseStamped()
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
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
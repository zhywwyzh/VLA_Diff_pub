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

@dataclass
class Frame:
    rgb_image: np.ndarray
    depth_image: np.ndarray
    current_state: np.ndarray
    stamp: float

class BasePolicyNode(Node):
    def __init__(self, node_name: str = "base_policy_node"):
        super().__init__(node_name)
        self.frame_lock = threading.Lock()
        self.frame: Optional[Frame] = None
        self.depth_info = None
        self.extrinsics = [0.35, 0, 0.1]
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
                                                       'camera/depth/info',
                                                       self.depth_info_callback,
                                                       10)

    def get_frame_snapshot(self) -> Optional[Frame]:
        with self.frame_lock:
            return self.frame

    def synced_callback(self, odom_msg: Odometry,
                    rgb_msg: CompressedImage,
                    depth_msg: CompressedImage):
        try:
            pos = odom_msg.pose.pose.position
            ori = odom_msg.pose.pose.orientation
            roll, pitch, yaw = self.quaternion_to_euler(ori.x, ori.y, ori.z, ori.w)
            current_state = np.array([pos.x-self.extrinsics[0], pos.y-self.extrinsics[1], pos.z-self.extrinsics[2], roll, pitch, yaw], dtype=np.float64)
            # print(f"机器人坐标为:{current_state}") 
            
            np_arr = np.frombuffer(rgb_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            rgb_image = cv_image.copy()

            np_arr = np.frombuffer(depth_msg.data, np.uint8)
            depth = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            depth_m = depth[:,:,0]
            depth_32 = depth_m.astype(np.float32) / 255.0 * 5.0
            depth_32[depth_32 == 0] = 5.0
            depth_image = depth_32.copy()

            stamp = Time.from_msg(odom_msg.header.stamp).nanoseconds / 1e9

            frame = Frame(rgb_image=rgb_image, depth_image=depth_image, 
                          current_state=current_state, stamp=stamp)
            with self.frame_lock:
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
        输入：检测结果(含bbox_2d)、帧快照(含depth/rgb/机器人位姿)
        输出：目标点在世界坐标下的三维位置 (x,y,z)
        """
        if self.depth_info is None:
            raise ValueError("相机参数未就绪")

        # 1) 像素 + 深度 → 相机系(OpenCV)
        u, v = self.bbox_center(results)
        fx, fy, cx, cy = (self.depth_info['fx'], self.depth_info['fy'],
                          self.depth_info['cx'], self.depth_info['cy'])

        H, W = frame.depth_image.shape[:2]
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < W and 0 <= vi < H):
            raise ValueError(f"(u,v)=({ui},{vi}) 超出图像范围 {W}x{H}")

        if window > 0:
            u0, u1 = max(0, ui - window), min(W - 1, ui + window)
            v0, v1 = max(0, vi - window), min(H - 1, vi + window)
            patch = frame.depth_image[v0:v1+1, u0:u1+1].astype(np.float64)
            depth_raw = np.median(patch)
        else:
            depth_raw = float(frame.depth_image[vi, ui])

        depth_val = depth_raw * depth_scale
        if not np.isfinite(depth_val) or depth_val <= 0:
            raise ValueError(f"无效深度 depth={depth_val}")

        P_cv = self._backproject(u, v, depth_val, fx, fy, cx, cy, depth_mode)

        # 2) 相机系 → 机器人系（坐标轴映射）
        # 机器人: x前 / y左 / z上; 相机(OpenCV): x右 / y下 / z前
        # x_r =  z_cv, y_r = -x_cv, z_r = -y_cv
        R_cam_to_robot = np.array([
            [ 0,  0,  1],
            [-1,  0,  0],
            [ 0, -1,  0]
        ], dtype=np.float64)

        # 加相机在机器人系的平移
        t_rc = self.extrinsics
        P_r = R_cam_to_robot @ P_cv + t_rc

        # 3) 机器人系 → 世界系
        tx, ty, tz, roll, pitch, yaw = map(float, frame.current_state.tolist())
        R_wr = self.euler_rpy_to_R(roll, pitch, yaw)     # 机器人→世界
        t_w = np.array([tx, ty, tz], dtype=np.float64)

        P_w = R_wr @ P_r + t_w
        return P_w
    
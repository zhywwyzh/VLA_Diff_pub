#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WebSocket relay for ROS1<->ROS2 via rosbridge.
Bridges:
  ROS1 -> ROS2:
    - /drone_0_visual_slam/odom            nav_msgs/Odometry
    - /camera/color/image/compressed       sensor_msgs/CompressedImage
    - /camera/depth/image/compressed       sensor_msgs/CompressedImage
  ROS2 -> ROS1:
    - /uav_actions                         geometry_msgs/PoseStamped

Fixes:
  - Remove header.seq when publishing to ROS2
  - secs/nsecs <-> sec/nanosec conversion
"""

import argparse
import time
import signal
import sys
from copy import deepcopy
import roslibpy

# --- Topics & Types ---
ODOM_TOPIC       = '/drone_0_visual_slam/odom'
IMG_RGB_TOPIC    = '/camera/color/image/compressed'
IMG_DEPTH_TOPIC  = '/camera/depth/image/compressed'
ACT_TOPIC        = '/uav_actions'

TYPE_ODOM = 'nav_msgs/Odometry'
TYPE_IMG  = 'sensor_msgs/CompressedImage'
TYPE_ACT  = 'geometry_msgs/PoseStamped'

# ----------------- Utils: JSON transform -----------------
def _recursive_map(obj, fn_dict=None):
    """Apply a dict-transform function recursively."""
    if isinstance(obj, dict):
        d = {k: _recursive_map(v, fn_dict) for k, v in obj.items()}
        return fn_dict(d) if fn_dict else d
    if isinstance(obj, list):
        return [_recursive_map(v, fn_dict) for v in obj]
    return obj

def _rename_time_keys(obj, mapping):
    """Rename time stamp keys recursively using mapping dict."""
    def _apply(d):
        newd = {}
        for k, v in d.items():
            nk = mapping.get(k, k)
            newd[nk] = v
        return newd
    return _recursive_map(obj, _apply)

def _drop_header_seq(obj):
    """Drop header.seq recursively (ROS2 std_msgs/Header has no 'seq')."""
    def _apply(d):
        hdr = d.get('header')
        if isinstance(hdr, dict):
            hdr.pop('seq', None)
        return d
    return _recursive_map(obj, _apply)

def to_ros2_json(msg_dict):
    """ROS1 -> ROS2: secs/nsecs -> sec/nanosec, drop header.seq."""
    x = deepcopy(msg_dict)
    x = _rename_time_keys(x, {'secs': 'sec', 'nsecs': 'nanosec'})
    x = _drop_header_seq(x)
    return x

def to_ros1_json(msg_dict):
    """ROS2 -> ROS1: sec/nanosec -> secs/nsecs (seq is optional, not added)."""
    x = deepcopy(msg_dict)
    x = _rename_time_keys(x, {'sec': 'secs', 'nanosec': 'nsecs'})
    return x

# ----------------- Bridge helpers -----------------
def bridge_ros1_to_ros2(ros1, ros2, topic_name, msg_type):
    sub = roslibpy.Topic(ros1, topic_name, msg_type)
    pub = roslibpy.Topic(ros2, topic_name, msg_type)
    pub.advertise()

    def _cb(msg):
        try:
            pub.publish(to_ros2_json(msg))
        except Exception as e:
            print(f'[ROS1→ROS2] publish error on {topic_name}: {e}')

    sub.subscribe(_cb)
    return sub, pub

def bridge_ros2_to_ros1(ros2, ros1, topic_name, msg_type):
    sub = roslibpy.Topic(ros2, topic_name, msg_type)
    pub = roslibpy.Topic(ros1, topic_name, msg_type)
    pub.advertise()

    def _cb(msg):
        try:
            pub.publish(to_ros1_json(msg))
        except Exception as e:
            print(f'[ROS2→ROS1] publish error on {topic_name}: {e}')

    sub.subscribe(_cb)
    return sub, pub

# ----------------- Main -----------------
def main():
    ap = argparse.ArgumentParser(description='ROS1<->ROS2 WebSocket relay via rosbridge')
    ap.add_argument('--ros1', default='0.0.0.0:10090', help='ROS1 rosbridge host:port')
    ap.add_argument('--ros2', default='0.0.0.0:10091', help='ROS2 rosbridge host:port')
    args = ap.parse_args()

    ros1_host, ros1_port = args.ros1.split(':')
    ros2_host, ros2_port = args.ros2.split(':')

    ros1 = roslibpy.Ros(host=ros1_host, port=int(ros1_port))
    ros2 = roslibpy.Ros(host=ros2_host, port=int(ros2_port))

    print('[*] Connecting to rosbridge servers...')
    ros1.run()
    ros2.run()
    if not ros1.is_connected:
        raise RuntimeError(f'Failed to connect ROS1 rosbridge at {args.ros1}')
    if not ros2.is_connected:
        raise RuntimeError(f'Failed to connect ROS2 rosbridge at {args.ros2}')
    print(f'[OK] Connected: ROS1({args.ros1}) & ROS2({args.ros2})')

    # Setup bridges
    subs_pubs = []
    subs_pubs += list(bridge_ros1_to_ros2(ros1, ros2, ODOM_TOPIC,      TYPE_ODOM))
    subs_pubs += list(bridge_ros1_to_ros2(ros1, ros2, IMG_RGB_TOPIC,   TYPE_IMG))
    subs_pubs += list(bridge_ros1_to_ros2(ros1, ros2, IMG_DEPTH_TOPIC, TYPE_IMG))
    subs_pubs += list(bridge_ros2_to_ros1(ros2, ros1, ACT_TOPIC,       TYPE_ACT))

    def cleanup(*_):
        print('\n[*] Stopping...')
        # unsubscribe first, then unadvertise
        for i in range(0, len(subs_pubs), 2):
            sub = subs_pubs[i]
            try:
                sub.unsubscribe()
            except Exception:
                pass
        for i in range(1, len(subs_pubs), 2):
            pub = subs_pubs[i]
            try:
                pub.unadvertise()
            except Exception:
                pass
        try:
            ros1.terminate()
        except Exception:
            pass
        try:
            ros2.terminate()
        except Exception:
            pass
        print('[*] Done.')
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    try:
        while ros1.is_connected and ros2.is_connected:
            time.sleep(0.001)
    except KeyboardInterrupt:
        cleanup()

if __name__ == '__main__':
    main()

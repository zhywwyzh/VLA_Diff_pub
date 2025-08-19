import os
import habitat
import signal
import numpy as np
from habitat.sims.habitat_simulator.actions import HabitatSimActions
from omegaconf import DictConfig
from habitat.config.default import patch_config
import hydra
from habitat2ros import habitat_publisher
import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.node import Node
from rclpy.duration import Duration
import threading
from copy import deepcopy
from std_msgs.msg import Int32, Int32MultiArray, Float32MultiArray, Float64
from geometry_msgs.msg import PoseStamped
from basic_utils.failure_check.failure_check import is_on_same_floor
import gzip, json, tqdm, time

from planner_interfaces.msg import MultipleMasksWithConfidence

from habitat.config.default_structured_configs import (
    CollisionsMeasurementConfig,
    FogOfWarConfig,
    TopDownMapMeasurementConfig,
)

from habitat.utils.visualizations.utils import (
    images_to_video,
    observations_to_image,
    overlay_frame,
)

from prettytable import PrettyTable
from basic_utils.record_episode.read_record import read_record
from basic_utils.record_episode.write_record import write_record
from basic_utils.failure_check.failure_check import check_failure
from basic_utils.failure_check.count_files import count_files_in_directory
from basic_utils.object_point_cloud_utils.object_point_cloud import (
    get_object_point_cloud,
)
from sensor_msgs.msg import PointCloud2
from vlm.utils.get_object_utils import get_object
from vlm.utils.get_itm_message import get_itm_message_cosine
from vlm.Labels import MP3D_ID_TO_NAME
from llm.answer_reader.answer_reader import read_answer
from params import HABITAT_STATE, ROS_STATE, ACTION, RESULT_TYPES


def signal_handler(sig, frame):
    print("Ctrl+C detected! Shutting down...")
    rclpy.signal_shutdown("Manual shutdown")
    os._exit(0)


def transform_rgb_bgr(image):
    return image[:, :, [2, 1, 0]]


class HabitatROSPublisher(Node):
    def __init__(self):
        super().__init__("habitat_ros_publisher")

        # Publisher
        self.itm_score_pub = self.create_publisher(Float64, "/blip2/cosine_score", 10)
        self.cld_with_score_pub = self.create_publisher(MultipleMasksWithConfidence, "/detector/clouds_with_scores", 10)
        self.confidence_threshold_pub = self.create_publisher(Float64, "/detector/confidence_threshold", 10)
        self.trigger_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.obj_point_cloud_pub = self.create_publisher(PointCloud2, "habitat/object_point_cloud", 10)
        self.state_pub = self.create_publisher(Int32, "/habitat/state", 10)
        self.progress_pub = self.create_publisher(Int32MultiArray, "/habitat/progress", 10)
        self.record_pub = self.create_publisher(Float32MultiArray, "/habitat/record", 10)

        # Subscriber
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Int32, "/habitat/plan_action", self.ros_action_callback, qos_profile)
        self.create_subscription(Int32, "/ros/state", self.ros_state_callback, qos_profile)
        self.create_subscription(Int32, "/ros/expl_state", self.ros_final_state_callback, qos_profile)
        self.create_subscription(Int32, "/ros/expl_result", self.ros_expl_result_callback, qos_profile)
        # self.create_subscription(Irclpynt32, "/habitat/plan_action", lambda msg: setattr(self, "global_action", msg.data), 10)
        # self.create_subscription(Int32, "/ros/state", lambda msg: setattr(self, "ros_state", msg.data), 10)
        # self.create_subscription(Int32, "/ros/expl_state", lambda msg: setattr(self, "final_state", msg.data), 10)
        # self.create_subscription(Int32, "/ros/expl_result", lambda msg: setattr(self, "expl_result", msg.data), 10)

        self.ros_pub = habitat_publisher.ROSPublisher()
        self.msg_observations = None
        self.fusion_score = 0.0
        self.global_action = None
        self.ros_state = ROS_STATE.INIT
        self.final_state = 0
        self.expl_result = 0
        self.timer_enabled = True

        self.pub_timer = self.create_timer(0.25, self.publish_observations)

    def ros_action_callback(self, msg):
        # print(f"Received action from ROS: {msg.data}")
        self.global_action = msg.data
        print(f"Received action from ROS: {self.global_action}")

    def ros_state_callback(self, msg):
        self.ros_state = msg.data
        # print(f"Received state from ROS: {self.ros_state}")

    def ros_final_state_callback(self, msg):
        self.final_state = msg.data
        # print(f"Received expl state from ROS: {self.final_state}")
        
    def ros_expl_result_callback(self, msg):
        self.expl_result = msg.data
        # print(f"Received expl state from ROS: {self.expl_result}")

    def publish_observations(self):
        if not self.timer_enabled:
            return
        if self.msg_observations is not None:
            tmp = deepcopy(self.msg_observations)
            self.ros_pub.habitat_publish_ros_topic(tmp)
            msg = Float64()
            msg.data = self.fusion_score
            self.confidence_threshold_pub.publish(msg)
            trigger = PoseStamped()
            self.trigger_pub.publish(trigger)


@hydra.main(
    version_base=None,
    config_path="config",
    config_name="habitat_eval_hm3d",
)
def main(cfg: DictConfig) -> None:    
    with gzip.open(
        "data/datasets/objectnav/mp3d/v1/val/val.json.gz", "rt", encoding="utf-8"
    ) as f:
        val_data = json.load(f)
    category_to_coco = val_data.get("category_to_mp3d_category_id", {})
    id_to_name = {
        category_to_coco[cat]: MP3D_ID_TO_NAME[idx]
        for idx, cat in enumerate(category_to_coco)
    }

    result_list = [0] * len(RESULT_TYPES)  # 确保 result_list 的长度和 RESULT_TYPES 一致

    # 统计数据
    start_time = time.time()

    cfg = patch_config(cfg)

    # 从cfg读取参数
    video_output_path = cfg.video_output_path.format(split=cfg.habitat.dataset.split)
    record_file_name = cfg.record_file_name
    file_path = os.path.join(video_output_path, record_file_name)
    continue_file_name = cfg.continue_file_name
    continue_path = os.path.join(video_output_path, continue_file_name)
    max_step = cfg.habitat.environment.max_episode_steps
    success_distance = cfg.habitat.task.measurements.success.success_distance
    
    detector_cfg = cfg.detector
    
    llm_cfg = cfg.llm
    llm_client = llm_cfg.llm_client
    llm_answer_path = llm_cfg.llm_answer_path
    llm_response_path = llm_cfg.llm_response_path
    # 单次测试参数
    flag_once = cfg.test_once  # 是否单次测试
    env_count = cfg.test_epi_num  # 单次测试哪个场景

    # 检查目录是否存在，不存在则创建
    os.makedirs(os.path.dirname(llm_answer_path), exist_ok=True)


    with habitat.config.read_write(cfg):
        cfg.habitat.task.measurements.update(
            {
                "top_down_map": TopDownMapMeasurementConfig(
                    map_padding=3,
                    map_resolution=256,
                    draw_source=True,
                    draw_border=True,
                    draw_shortest_path=True,
                    draw_view_points=True,
                    draw_goal_positions=True,
                    draw_goal_aabbs=False,
                    fog_of_war=FogOfWarConfig(
                        draw=True,
                        visibility_dist=5.0,
                        fov=79,
                    ),
                ),
                "collisions": CollisionsMeasurementConfig(),
            }
        )

    env = habitat.Env(cfg)
    print("Environment creation successful")
    number_of_episodes = env.number_of_episodes

    # 读取record并设置初始值
    (
        num_total,
        num_success,
        spl_all,
        soft_spl_all,
        distance_to_goal_all,
        distance_to_goal_reward_all,
        last_time,
    ) = read_record(continue_path, flag_once)

    if num_total >= number_of_episodes:
        raise ValueError("Already finished all episodes.")

    pbar = tqdm.tqdm(total=env.number_of_episodes)

    env_count = num_total
    while env_count:
        pbar.update()
        env.current_episode = next(env.episode_iterator)
        env_count -= 1

    for epi in range(number_of_episodes - num_total):
        progress_msg = Int32MultiArray()
        progress_msg.data = [num_total, number_of_episodes]
        node.progress_pub.publish(progress_msg)

        pass_object = 0.0
        near_object = 0.0
        camera_pitch = 0.0
        if flag_once:
            while env_count:
                env.current_episode = next(env.episode_iterator)
                env_count -= 1

        observations = env.reset()
        observations["camera_pitch"] = camera_pitch
        node.msg_observations = deepcopy(observations)
        del observations["camera_pitch"]
        label = env.current_episode.object_category

        if label in category_to_coco:
            coco_id = category_to_coco[label]
            label = id_to_name.get(coco_id, label)

        llm_answer, room, fusion_score = read_answer(
            llm_answer_path, llm_response_path, label, llm_client
        )

        node.fusion_score = fusion_score

        cld_with_score_msg = MultipleMasksWithConfidence()
        cld_with_score_msg.point_clouds = []
        cld_with_score_msg.confidence_scores = []
        cld_with_score_msg.label_indices = []

        # For generate video
        info = env.get_metrics()
        frame = observations_to_image(observations, info)

        info.pop("top_down_map")
        frame = overlay_frame(frame, info)
        vis_frames = [frame]

        print("Agent stepping around inside environment.")

        # 等待ROS准备好
        while node.ros_state == ROS_STATE.INIT or node.ros_state == ROS_STATE.WAIT_TRIGGER:
            if node.ros_state == ROS_STATE.INIT:
                print("Wait ROS get odom")
            elif node.ros_state == ROS_STATE.WAIT_TRIGGER:
                print("Wait ROS get trigger")
            # rate.sleep()
            # rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)

        # 开始执行action则关闭定时发布仿真信息和trigger
        node.timer_enabled = False
        node.pub_timer.destroy()
        
        count_steps = 0
        while rclpy.ok() and not env.episode_over:
            is_feasible = 0
            for goal in env.current_episode.goals:
                height = goal.position[1]
                is_feasible += is_on_same_floor(
                    height=height, episode=env.current_episode
                )
            if not is_feasible:
                break

            # 读取决策得到的动作
            action = None
            if node.global_action is not None:
                if node.global_action == ACTION.MOVE_FORWARD:
                    action = HabitatSimActions.move_forward
                elif node.global_action == ACTION.TURN_LEFT:
                    action = HabitatSimActions.turn_left
                elif node.global_action == ACTION.TURN_RIGHT:
                    action = HabitatSimActions.turn_right
                elif node.global_action == ACTION.TURN_DOWN:
                    action = HabitatSimActions.look_down
                    camera_pitch = camera_pitch - np.pi / 6.0
                elif node.global_action == ACTION.TURN_UP:
                    action = HabitatSimActions.look_up
                    camera_pitch = camera_pitch + np.pi / 6.0
                elif node.global_action == ACTION.STOP:
                    action = HabitatSimActions.stop

                node.global_action = None

            if action is None:
                continue

            state_msg = Int32()
            state_msg.data = HABITAT_STATE.ACTION_EXEC
            node.state_pub.publish(state_msg)

            print(f"I'm finding {label}")
            observations = env.step(action)

            # 需要调整逻辑
            print(f"room: {room}")
            # cosine = get_itm_message_cosine(observations["rgb"], label, room)
            cosine = 0.0
            print(f"out_cosine: {cosine:.3f}")
            itm_score_msg = Float64()
            itm_score_msg.data = cosine
            node.itm_score_pub.publish(itm_score_msg)

            # observations["rgb"], score_list, object_masks_list, label_list = get_object(
            #     label, observations["rgb"], detector_cfg, llm_answer
            # )
            object_masks_list = []
            score_list = []
            label_list = []

            observations["camera_pitch"] = camera_pitch
            node.msg_observations = deepcopy(observations)
            del observations["camera_pitch"]
            node.ros_pub.habitat_publish_ros_topic(node.msg_observations)
            obj_point_cloud_list = get_object_point_cloud(
                cfg, observations, object_masks_list
            )
            count_steps += 1

            # 发布检测相关的信息
            cld_with_score_msg.point_clouds = obj_point_cloud_list
            cld_with_score_msg.confidence_scores = score_list
            cld_with_score_msg.label_indices = label_list
            node.cld_with_score_pub.publish(cld_with_score_msg)

            # For generate video
            info = env.get_metrics()
            frame = observations_to_image(observations, info)

            distance_to_goal = info["distance_to_goal"]
            if distance_to_goal <= success_distance and pass_object == 0:
                pass_object = 1

            info.pop("top_down_map")
            frame = overlay_frame(frame, info)
            vis_frames.append(frame)

            # 发布消息告诉expl_node本次action已经执行完，正在等待下一个action
            state_msg = Int32()
            state_msg.data = HABITAT_STATE.ACTION_FINISH
            node.state_pub.publish(state_msg)
            print(f"step: {count_steps}; action: {action}")
            time.sleep(1.0)

        # 发布消息告诉expl_node当前剧集已经评估完成了，需要初始化进行下一个剧集的评估
        state_msg = Int32()
        state_msg.data = HABITAT_STATE.EPISODE_FINISH
        node.state_pub.publish(state_msg)

        # 统计评估数据
        info = env.get_metrics()
        spl = info["spl"]
        soft_spl = info["soft_spl"]
        distance_to_goal = info["distance_to_goal"]
        distance_to_goal_reward = info["distance_to_goal_reward"]
        success = info["success"]

        final_state = node.final_state
        expl_result = node.expl_result

        if distance_to_goal <= success_distance:
            near_object = 1

        if success == 1:
            num_success += 1
            result_text = "success"
        else:
            result_text = check_failure(
                env.current_episode,
                final_state,
                expl_result,
                count_steps,
                max_step,
                pass_object,
                near_object,
            )

        num_total += 1
        spl_all += spl
        soft_spl_all += soft_spl
        distance_to_goal_all += distance_to_goal
        distance_to_goal_reward_all += distance_to_goal_reward

        scene_id = env.current_episode.scene_id
        episode_id = env.current_episode.episode_id
        video_name = f"{os.path.basename(scene_id)}_{episode_id}"
        time_spend = time.time() - start_time + last_time

        img2video_output_path = os.path.join(video_output_path, result_text)

        if flag_once:
            img2video_output_path = "videos"
            video_name = "video_once"

        images_to_video(vis_frames, img2video_output_path, video_name, fps=6, quality=9)
        vis_frames.clear()

        # 输出平均数据
        table1 = PrettyTable(["Metric", "Average"])
        table1.add_row(["Average Success", f"{num_success/num_total * 100:.2f}%"])
        table1.add_row(["Average SPL", f"{spl_all/num_total * 100:.2f}%"])
        table1.add_row(["Average Soft SPL", f"{soft_spl_all/num_total * 100:.2f}%"])
        table1.add_row(
            ["Average Distance to Goal", f"{distance_to_goal_all/num_total:.4f}"]
        )
        print(table1)
        print(f"No.{num_total} task's data is written to the {file_path}")
        print(f"success or not: {result_text}")

        # 输出总数据
        table2 = PrettyTable(["Metric", "Total"])
        table2.add_row(["Total Success", f"{num_success}"])
        table2.add_row(["Total SPL", f"{spl_all:.2f}"])
        table2.add_row(["Total Soft SPL", f"{soft_spl_all:.2f}"])
        table2.add_row(["Total Distance to Goal", f"{distance_to_goal_all:.4f}"])

        if flag_once:
            break

        # 输出内容到record文件
        write_record(
            scene_id,
            episode_id,
            table1,
            result_text,
            label,
            num_total,
            time_spend,
            file_path,
        )

        # 输出内容到continue文件
        write_record(
            scene_id,
            episode_id,
            table2,
            result_text,
            label,
            num_total,
            time_spend,
            continue_path,
        )

        for i in range(len(RESULT_TYPES)):
            folder = RESULT_TYPES[i]  # 获取当前类型（文件夹名）
            folder_path = os.path.join(video_output_path, folder)  # 拼接文件夹路径
            file_count = count_files_in_directory(folder_path)  # 获取该文件夹中的文件数
            result_list[i] = file_count

        record_msg = Float32MultiArray()
        record_msg.data = [
            num_success / num_total * 100,
            spl_all / num_total * 100,
            soft_spl_all / num_total * 100,
            distance_to_goal_all / num_total,
        ]
        record_msg.data.extend(result_list)
        node.record_pub.publish(record_msg)

        # rclpy.spin_once(node)
        pbar.update()
        env.current_episode = next(env.episode_iterator)

    env.close()
    pbar.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    # init ros2 
    rclpy.init()
    node = HabitatROSPublisher()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    try:
        main()
    except Exception as e:
        print(f"Unexpected error occurred: {e}")
        rclpy.shutdown()
        os._exit(1)

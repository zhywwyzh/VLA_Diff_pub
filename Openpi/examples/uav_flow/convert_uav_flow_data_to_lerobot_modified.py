"""
Minimal example script for converting a custom UAV-Flow dataset (from Parquet files) to LeRobot format.

The script reads Parquet files from the specified data directory, processes them episode by episode
(grouped by 'id'), and converts them into LeRobot's format.

Usage:
uv run examples/UAV-FLOW/convert_uav_flow_data_to_lerobot.py --data_dir /path/to/your/uav_parquet_files_folder

If you want to push your dataset to the Hugging Face Hub, you can use the following command:
uv run examples/UAV-FLOW/convert_uav_flow_data_to_lerobot.py --data_dir /path/to/your/uav_parquet_files_folder --push_to_hub

Note: to run the script, you need to install pyarrow and Pillow:
`uv pip install pyarrow Pillow`

The resulting dataset will get saved to the $HF_LEROBOT_HOME directory.
"""

import shutil
import os
import glob
import pyarrow.parquet as pq
import numpy as np
from PIL import Image
import io 
import json 
import logging 
import gc # Import the garbage collection module
import psutil
import traceback
import sys

from lerobot.common.datasets.lerobot_dataset import HF_LEROBOT_HOME
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
import tyro

# Configuration for the new dataset
REPO_NAME = "uav_flow"  # Name of the output dataset, also used for the Hugging Face Hub. You can change this.
TARGET_IMAGE_HEIGHT = 256  # Target height for images after resizing. Adjust as needed.
TARGET_IMAGE_WIDTH = 256   # Target width for images after resizing. Adjust as needed.
FPS = 5          # 原文中提到uniformly sample the video at 5 Hz
MAX_EPISODES_TO_PROCESS = 60000 # Set to a small number for testing, or None to process all episodes
PARQUET_FILE_PREFIX = "pass" # Prefix for the parquet files to be processed.


def process_parquet_file(pq_file, logger, processed_episodes, episodes_processed_count, MAX_EPISODES_TO_PROCESS, output_dir, raw_metadata):
    """
    Processes a single Parquet file, extracts episodes, and adds them to the LeRobot dataset.
    Returns the updated count of processed episodes and a flag indicating if any episode was saved.
    """
    any_episode_saved_from_file = False
    table = None
    try:
        # Force garbage collection before loading new file
        gc.collect()
        table = pq.read_table(pq_file)
        logger.debug(f"File {pq_file} loaded with {table.num_rows} rows.")
    except Exception as e:
        logger.error(f"Error reading Parquet file {pq_file}: {e}. Skipping this file.")
        return episodes_processed_count, any_episode_saved_from_file

    if table.num_rows == 0:
        logger.warning(f"File {pq_file} is empty. Skipping.")
        del table
        gc.collect()
        return episodes_processed_count, any_episode_saved_from_file

    # Sort by id and frame_idx to ensure correct order within episodes for this file.
    table = table.sort_by([('id', 'ascending'), ('frame_idx', 'ascending')])

    # Manually group by 'id' to process each episode within the current file
    if table.num_rows > 0:
        episode_data = table.to_pylist()
        del table # Free memory from the pyarrow table
        gc.collect()

        episode_groups = []
        current_episode_id = None
        current_episode_group = []
        for row in episode_data:
            row_episode_id = row['id']
            if current_episode_id != row_episode_id:
                if current_episode_group:
                    episode_groups.append((current_episode_id, current_episode_group))
                current_episode_id = row_episode_id
                current_episode_group = []
            current_episode_group.append(row)
        
        # Add the last group
        if current_episode_group:
            episode_groups.append((current_episode_id, current_episode_group))

        del episode_data # Free memory from the list of dicts
        gc.collect()
    else:
        episode_groups = []

    logger.info(f"Processing {len(episode_groups)} episodes from '{os.path.basename(pq_file)}'. Memory usage: {psutil.Process(os.getpid()).memory_info().rss / (1024 * 1024):.2f}MB")

    for episode_id, episode_objects in episode_groups:
        if str(episode_id) in processed_episodes:
            logger.debug(f"Skipping already processed episode '{episode_id}'.")
            continue

        if MAX_EPISODES_TO_PROCESS is not None and episodes_processed_count >= MAX_EPISODES_TO_PROCESS:
            logger.info(f"Reached MAX_EPISODES_TO_PROCESS limit ({MAX_EPISODES_TO_PROCESS}) while processing episodes in {pq_file}. Stopping.")
            break

        # Create a new dataset object for each episode to ensure memory is released
        dataset = LeRobotDataset(
            dataset_dir=output_dir,
            hf_repo_id=f"{os.getenv('HF_USERNAME', 'lerobot')}/{REPO_NAME}",
            episode_data_format="hdf5",
            raw_metadata=raw_metadata,
            episode_metadata_format="json",
            info_updates={
                "fps": FPS,
                "video": False,
            },
            camera_names=["image", "wrist_image"],
            observation_info={
                "state": {
                    "dtype": "float32",
                    "shape": (6,),
                    "names": ["state"],
                },
            },
            action_info={
                "actions": {
                    "dtype": "float32",
                    "shape": (6,),
                    "names": ["actions"],
                },
            },
            image_writer_threads=4,
            image_writer_processes=2,
        )

        # episode_objects is now the list of dicts from the group
        num_objects_in_episode = len(episode_objects)

        logger.debug(f"Processing episode '{episode_id}' with {num_objects_in_episode} objects.")

        if num_objects_in_episode < 2:
            logger.warning(f"Episode '{episode_id}' has {num_objects_in_episode} objects, less than the required 2. Skipping.")
            continue

        try:
            raw_common_log = episode_objects[0]['log']
            common_log = json.loads(raw_common_log) if isinstance(raw_common_log, str) else raw_common_log
            if not isinstance(common_log, dict):
                logger.error(f"Episode '{episode_id}': 'log' field is not a dictionary or valid JSON string. Skipping.")
                continue
            
            task_instruction = common_log.get('instruction_unified', "")
            preprocessed_logs_list = common_log.get('preprocessed_logs')

            if preprocessed_logs_list is None:
                logger.error(f"Episode '{episode_id}': 'preprocessed_logs' missing in 'log' data. Skipping.")
                continue
            
            if len(preprocessed_logs_list) != num_objects_in_episode:
                logger.error(f"Episode '{episode_id}': Mismatch in log list length and number of objects. Skipping.")
                continue
        except (KeyError, json.JSONDecodeError, Exception) as e:
            logger.error(f"Episode '{episode_id}': Error processing 'log' data: {e}. Skipping episode.")
            continue

        pil_first_image = None
        first_image_arr = None
        try:
            raw_first_image_data = episode_objects[0]['image']
            first_image_bytes = raw_first_image_data.get('bytes') if isinstance(raw_first_image_data, dict) else raw_first_image_data
            if not isinstance(first_image_bytes, bytes) or not first_image_bytes:
                logger.error(f"Episode '{episode_id}': First image data is not valid. Skipping episode.")
                continue
            pil_first_image = Image.open(io.BytesIO(first_image_bytes)).convert("RGB").resize((TARGET_IMAGE_WIDTH, TARGET_IMAGE_HEIGHT))
            first_image_arr = np.array(pil_first_image, dtype=np.uint8)
            pil_first_image.close()  # Explicitly close PIL image
            pil_first_image = None
        except Exception as e:
            logger.error(f"Episode '{episode_id}': Error processing first image: {e}. Skipping episode.")
            if pil_first_image:
                pil_first_image.close()
            continue
        
        frames_added_to_this_episode = 0
        for step_idx in range(num_objects_in_episode - 1):
            pil_wrist_image = None
            wrist_image_arr = None
            try:
                raw_current_image_data = episode_objects[step_idx]['image']
                current_image_bytes = raw_current_image_data.get('bytes') if isinstance(raw_current_image_data, dict) else raw_current_image_data
                if not isinstance(current_image_bytes, bytes) or not current_image_bytes:
                    logger.warning(f"Episode '{episode_id}', Step {step_idx}: Wrist image data is not valid. Skipping step.")
                    continue
                pil_wrist_image = Image.open(io.BytesIO(current_image_bytes)).convert("RGB").resize((TARGET_IMAGE_WIDTH, TARGET_IMAGE_HEIGHT))
                wrist_image_arr = np.array(pil_wrist_image, dtype=np.uint8)
                pil_wrist_image.close()  # Explicitly close PIL image
                pil_wrist_image = None

                state_data = np.array(preprocessed_logs_list[step_idx], dtype=np.float32)
                action_data = np.array(preprocessed_logs_list[step_idx + 1], dtype=np.float32)
                if state_data.shape != (6,) or action_data.shape != (6,):
                    logger.warning(f"Episode '{episode_id}', Step {step_idx}: Incorrect state/action shape. Skipping step.")
                    continue

                frame_payload = {
                    "image": first_image_arr,
                    "wrist_image": wrist_image_arr,
                    "state": state_data,
                    "actions": action_data,
                    "task": task_instruction,
                }
                dataset.add_frame(frame_payload)
                frames_added_to_this_episode += 1
                
                # Clean up arrays after adding frame
                del wrist_image_arr, state_data, action_data, frame_payload
            except Exception as e:
                logger.warning(f"Episode '{episode_id}', Step {step_idx}: Error processing step: {e}. Skipping step.")
                if pil_wrist_image:
                    pil_wrist_image.close()
                continue

        if frames_added_to_this_episode > 0:
            dataset.save_episode()
            logger.debug(f"Successfully saved episode '{episode_id}' with {frames_added_to_this_episode} steps.")
            episodes_processed_count += 1
            processed_episodes[str(episode_id)] = task_instruction
            logger.info(f"Total episodes processed: {episodes_processed_count}")
            any_episode_saved_from_file = True
        else:
            logger.warning(f"No frames were added for episode '{episode_id}', so it was not saved.")
        
        # Clean up episode-level variables
        del first_image_arr, episode_objects
        if 'preprocessed_logs_list' in locals():
            del preprocessed_logs_list

        if episodes_processed_count % 10 == 0:
            gc.collect()

    # Clean up to free memory
    del episode_groups
    gc.collect()
    
    return episodes_processed_count, any_episode_saved_from_file


def main(data_dir: str, *, push_to_hub: bool = False):
    # Set up logging
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG) # Set base level for the logger to capture all desired levels

    # Clear existing handlers if any (e.g., during re-runs in an interactive session)
    if logger.hasHandlers():
        logger.handlers.clear()

    # Create file handler for error logs
    error_log_path = "/data/vla/Openpi/examples/uav_flow/error.log"
    try:
        os.makedirs(os.path.dirname(error_log_path), exist_ok=True)
        file_handler = logging.FileHandler(error_log_path, mode='w') # 'w' to overwrite log file on each run
    except OSError as e:
        print(f"Warning: Could not create directory for error log file {error_log_path}: {e}. Errors will not be saved to file.")
        file_handler = None

    if file_handler:
        file_handler.setLevel(logging.WARNING)
        file_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s')
        file_handler.setFormatter(file_formatter)
        logger.addHandler(file_handler)

    # Create console handler for non-error logs (INFO, WARNING)
    console_handler = logging.StreamHandler()
    # console_handler.setLevel(logging.INFO) # Log INFO and WARNING to console
    # Filter out WANRING and ERROR and CRITICAL for console handler
    # console_handler.addFilter(lambda record: record.levelno < logging.WARNING) # Temporarily disable or modify this filter
    console_handler.setLevel(logging.INFO) # Show INFO and WARNING logs on console
    console_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s') # More detailed format
    console_handler.setFormatter(console_formatter)
    logger.addHandler(console_handler)

    try:
        # Clean up any existing dataset in the output directory
        output_path = HF_LEROBOT_HOME / REPO_NAME
        if output_path.exists():
            logger.info(f"Cleaning up existing dataset at {output_path}")
            # shutil.rmtree(output_path) # Consider commenting this out if you want to debug continuation
        # output_path.mkdir(parents=True, exist_ok=True) # Remove this line

        # Create LeRobot dataset directory and metadata once
        LeRobotDataset.create(
            repo_id=REPO_NAME,
            robot_type="uav",
            fps=FPS,
            features={
                "image": {
                    "dtype": "image",
                    "shape": (TARGET_IMAGE_HEIGHT, TARGET_IMAGE_WIDTH, 3),
                    "names": ["height", "width", "channel"],
                },
                "wrist_image": {
                    "dtype": "image",
                    "shape": (TARGET_IMAGE_HEIGHT, TARGET_IMAGE_WIDTH, 3),
                    "names": ["height", "width", "channel"],
                },
                "state": {
                    "dtype": "float32",
                    "shape": (6,),
                    "names": ["state"],
                },
                "actions": {
                    "dtype": "float32",
                    "shape": (6,),
                    "names": ["actions"],
                },
            },
            image_writer_threads=4,
            image_writer_processes=2,
        )

        raw_metadata = {
            "fps": FPS,
            "video": False,
        }

        # Find and load all Parquet files
        file_pattern = f"*.parquet"
        parquet_files = sorted(glob.glob(os.path.join(data_dir, file_pattern)))
        if not parquet_files:
            logger.error(f"No Parquet files found in '{data_dir}' matching '{file_pattern}'. Please check the path.")
            return

        logger.info(f"Found {len(parquet_files)} Parquet files to process.")

        episodes_processed_count = 0
        processed_episodes = {}
        logger.info(f"Loaded {len(processed_episodes)} processed episodes from checkpoint.")

        for pq_file in parquet_files:
            if episodes_processed_count >= MAX_EPISODES_TO_PROCESS:
                logger.info(f"Reached MAX_EPISODES_TO_PROCESS limit of {MAX_EPISODES_TO_PROCESS}. Stopping.")
                break
            episodes_processed_count, _ = process_parquet_file(pq_file, logger, processed_episodes, episodes_processed_count, MAX_EPISODES_TO_PROCESS, output_path, raw_metadata)

        logger.info("Dataset conversion process finished.")
        if not any(episodes_processed_count):
            logger.warning("No episodes were successfully processed and saved from any of the files.")
        else:
            logger.info(f"Successfully processed and saved {episodes_processed_count} episode(s) in total.")
            if MAX_EPISODES_TO_PROCESS is not None and episodes_processed_count < MAX_EPISODES_TO_PROCESS:
                logger.info(f"Note: The MAX_EPISODES_TO_PROCESS limit was {MAX_EPISODES_TO_PROCESS}, but only {episodes_processed_count} were processed. This could be due to reaching the end of available data or episodes being skipped due to errors/filters.")

        # Consolidate the dataset (optional, can be done later)
        # logger.info("Consolidating dataset...")
        # dataset.consolidate(run_compute_stats=False) # Set run_compute_stats=True if you want stats computed now

        # Optionally push to the Hugging Face Hub
        if push_to_hub:
            logger.info(f"Pushing dataset {REPO_NAME} to Hugging Face Hub...")
            LeRobotDataset.push_to_hub(
                repo_id=REPO_NAME, # Ensure repo_id is passed if it's different from the one in create() or for clarity
                tags=["uav-flow", "uav", "custom-dataset"], # Add relevant tags
                private=False,  # Set to True if you want a private dataset on the Hub
                push_videos=True, # If you want to generate and push video previews
                license="apache-2.0", # Or your preferred license
            )
            logger.info("Dataset pushed to Hub.")

    except Exception as e:
        logger.error("Unhandled exception occurred in main:", exc_info=True)
        # Write detailed exception info to a separate file for thorough debugging
        with open(os.path.join(os.path.dirname(error_log_path), "crash_details.txt"), "w") as crash_file:
            crash_file.write(f"Exception: {str(e)}\n\n")
            crash_file.write(f"Stack trace:\n{traceback.format_exc()}\n\n")
            crash_file.write(f"Python version: {sys.version}\n")
            crash_file.write(f"Memory usage at crash: RSS={psutil.Process(os.getpid()).memory_info().rss / (1024 * 1024):.2f}MB\n")
        raise  # Re-raise to ensure proper exit code
    finally:
        logging.shutdown() # Ensure all handlers are flushed


if __name__ == "__main__":
    tyro.cli(main)

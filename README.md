# VLA_Diff
VLA code in Diffrobot

## Installation

```bash
git clone git@github.com:Wyz000/VLA_Diff.git

```

We use [uv](https://docs.astral.sh/uv/) to manage Python dependencies. See the [uv installation instructions](https://docs.astral.sh/uv/getting-started/installation/) to set it up. Once uv is installed, run the following to set up the environment:

```bash
GIT_LFS_SKIP_SMUDGE=1 uv sync
GIT_LFS_SKIP_SMUDGE=1 uv pip install -e .
```

NOTE: `GIT_LFS_SKIP_SMUDGE=1` is needed to pull LeRobot as a dependency.

## Modify the dependent code:

1. In the file `.venv/lib/python3.11/site-packages/lerobot/common/datasets/lerobot_dataset.py`:
   - Comment out the following lines:
     ```python
     # self.hf_dataset = concatenate_datasets([self.hf_dataset, ep_dataset])
     # self.hf_dataset.set_transform(hf_transform_to_torch)
     ```
     Otherwise, it may cause out-of-memory (OOM) errors.
   - Change:
     ```python
     ep_data_index = get_episode_data_index(self.meta.episodes, [episode_index])
     ```
     to:
     ```python
     ep_data_index = get_episode_data_index({episode_index: {"length": episode_length}}, [episode_index])
     ```
   - Comment out the following lines:
     ```python
     # self.episodes[episode_index] = episode_dict
     # self.episodes_stats[episode_index] = episode_stats
     ```

2. In the file `.venv/lib/python3.11/site-packages/lerobot/common/datasets/utils.py`, change `DEFAULT_CHUNK_SIZE` to 100.

## Fine-Tuning Base Models on Your Own Data
### 1. Convert uav-flow raw data to a LeRobot dataset

uv run VLA_Diff/Openpi/examples/uav_flow/convert_uav_flow_data_to_lerobot.py --data_dir /path/to/your/libero/data

### 2. Defining training configs and running training
Before we can run training, we need to compute the normalization statistics for the training data. Run the script below with the name of your training config:

```bash
uv run VLA_Diff/Openpi/scripts/compute_norm_stats_cuda.py --config-name pi0_uav_low_mem_finetune
```

Now we can kick off training with the following command (the `--overwrite` flag is used to overwrite existing checkpoints if you rerun fine-tuning with the same config; If you want to continue training from a specific checkpoint, use the `--resume` flag.):

```bash
XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 uv run VLA_Diff/Openpi/scripts/train.py --exp-name=my_experiment --resume
```

The command will log training progress to the console and save checkpoints to the `checkpoints` directory. You can also monitor training progress on the Weights & Biases dashboard. For maximally using the GPU memory, set `XLA_PYTHON_CLIENT_MEM_FRACTION=0.9` before running training -- this enables JAX to use up to 90% of the GPU memory (vs. the default of 75%).

## Spinning up a policy server and running inference

Once training is complete, we can run inference by spinning up a policy server and then querying it from a Libero evaluation script. Launching a model server is easy (we use the checkpoint for iteration 20,000 for this example, modify as needed):

```bash
uv run VLA_Diff/Openpi/scripts/serve_policy.py policy:checkpoint --policy.config=pi0_uav_low_mem_finetune --policy.dir=checkpoints/pi0_uav_low_mem_finetune/pi0_uav_low_mem_finetune_3w_0801/100000
```

This will spin up a server that listens on port 8000 and waits for observations to be sent to it.
Then run`VLA_Diff/Openpi/test/infer/evaluate_with_parquet.ipynb`
for inference. The dataset and the pre-trained model checkpoint can be downloaded from `LXX3123` on ModelScope.

## Set Up Simulation Environment
### Notes:
- Make sure all dependencies are installed before running these commands
  - Unity Editor: Version 2022.3.61f1c1 (recommended for full compatibility)
  - ROS: Noetic distribution installed and configured
- Replace .zsh with .bash if you're using bash shell

### 1. Download Models

Run the following script to download required models:

```bash
Waiting for completion...
```

Open the downloaded models with unity and run the simulation scene.

### 2. Setup ROS-Unity Bridge

```bash
cd VLA_Diff/simulation/ROS-Unity_bridge
catkin_make    # Only needed if you haven't built before
source devel/setup.zsh
roslaunch ros_tcp_endpoint endpoint.launch
```

### 3. Run EGO-Planner


```bash
cd VLA_Diff/simulation/EGO-Planner-v2
catkin_make    # Only needed if you haven't built before
source devel/setup.zsh
roslaunch ego_planner single_drone_interactive.launch
```

### 4. Spinning up a policy server and running inference

```bash
cd VLA_Diff/Openpi
source .venv/bin/activate # Only needed if you are not in the environment of openpi
uv run VLA_Diff/Openpi/scripts/serve_policy.py policy:checkpoint --policy.config=pi0_uav_low_mem_finetune --policy.dir=checkpoints/pi0_uav_low_mem_finetune/pi0_uav_low_mem_finetune_3w_0801/100000
```

### 5. Simulation testing

```bash
cd VLA_Diff/Openpi/test/infer
source .venv/bin/activate # Only needed if you are not in the environment of openpi
python test_simulation.py
```

### 6. Run goal-utils and cloud_filter
```bash
cd VLA/simulation/EGO-Planner-v3
catkin_make
source devel/setup.bash
roslaunch cloud_filter filter.launch
```

### 7. Run EGO-Planner-v3

```bash
cd VLA/simulation/EGO-Planner-v3
catkin_make
source devel/setup.bash
roslaunch mission_fsm multidrone_sim.launch
roslaunch mission_fsm rviz.launch
```

### 8. Run vllm
```bash
vllm serve /home/zhywwyzh/Modelscope/qwen2.5-vl-7B-Instruct-AWQ --dtype auto --port 6006 --max-model-len 3000 --gpu-memory-utilization 0.8
```

### 9. Run test in simulation-ros2
```bash
cd Openpi
source .
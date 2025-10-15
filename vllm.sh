export CUDA_VISIBLE_DEVICES=0

vllm serve /home/zhywwyzh/workspace/VLA_Diff/output/3dgs/7B/new/mission_sharegpt_oreder_shuffle_awq \
  --dtype auto \
  --port 9000 \
  --max-model-len 3000 \
  --gpu-memory-utilization 0.6 \
  --max-num-seqs 1 \
  --enforce-eager
export CUDA_VISIBLE_DEVICES=0

vllm serve /home/diff/workspace/VLA_Diff/output/mission_sharegpt_qwen25_freeze_order_skip_empty_awq \
  --dtype auto \
  --port 4514 \
  --max-model-len 3000 \
  --gpu-memory-utilization 0.95 \
  --max-num-seqs 1 \
  --enforce-eager
export CUDA_VISIBLE_DEVICES=0

vllm serve /home/zhywwyzh/workspace/VLA_Diff/output/3dgs-new/7B/mission_sharegpt_qwen25_sample3_order_with_none_awq \
  --dtype auto \
  --port 9000 \
  --max-model-len 3000 \
  --gpu-memory-utilization 0.6 \
  --max-num-seqs 1 \
  --enforce-eager
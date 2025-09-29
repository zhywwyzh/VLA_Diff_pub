vllm serve /home/zhywwyzh/workspace/LLaMA-Factory/output/3dgs/7B/full_sft_all_unfreeze_shuffle_fix_mission_awq \
  --dtype auto \
  --port 9000 \
  --max-model-len 4096 \
  --gpu-memory-utilization 0.8 \
  --max-num-seqs 1
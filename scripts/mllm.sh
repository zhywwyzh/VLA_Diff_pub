cd ~/workspace/VLA_Diff;
conda deactivate
source venv/bin/activate
cd pipeline

unset all_proxy; unset ALL_PROXY;

./start_all.sh # 启动

./stop_all.sh # 停止
cd ~/workspace/VLA_Diff;
conda deactivate
source venv/bin/activate
cd pipeline

unset all_proxy; unset ALL_PROXY;
# unset http_proxy; unset HTTP_PROXY
# unset https_proxy; unset HTTPS_PROXY

# ./stop_all.sh # 停止
./start_all.sh # 启动
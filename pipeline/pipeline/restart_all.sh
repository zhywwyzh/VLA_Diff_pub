#!/bin/bash

# Gemini 多模态聊天系统 - 重启所有服务

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_header() {
    echo -e "${PURPLE}$1${NC}"
}

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

clear
log_header "============================================================"
log_header "    🔄 重启 Gemini 多模态聊天系统服务"
log_header "============================================================"
echo ""

# 检查必要脚本是否存在
if [ ! -f "stop_all.sh" ]; then
    log_error "找不到 stop_all.sh 脚本"
    exit 1
fi

if [ ! -f "start_all.sh" ]; then
    log_error "找不到 start_all.sh 脚本"
    exit 1
fi

# 确保脚本有执行权限
chmod +x stop_all.sh start_all.sh

log_info "开始重启服务..."
echo ""

# 第一步：停止所有服务
log_header "第一步：停止现有服务"
echo ""

./stop_all.sh

if [ $? -ne 0 ]; then
    log_error "停止服务时出现错误"
    read -p "$(echo -e ${YELLOW}"是否继续启动服务？(y/N): "${NC})" continue_start
    if [[ ! $continue_start =~ ^[Yy]$ ]]; then
        log_info "重启已取消"
        exit 1
    fi
fi

echo ""
log_info "等待服务完全停止..."
sleep 3

# 第二步：启动所有服务  
log_header "第二步：启动服务"
echo ""

./start_all.sh

if [ $? -ne 0 ]; then
    log_error "启动服务时出现错误"
    exit 1
fi

log_header "🎉 服务重启完成！"

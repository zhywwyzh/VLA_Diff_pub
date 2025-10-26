#!/bin/bash

# Gemini 多模态聊天系统 - 停止所有服务

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
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

log_header "============================================================"
log_header "    🛑 停止 Gemini 多模态聊天系统服务"
log_header "============================================================"
echo ""

# 停止指定PID的进程
stop_process() {
    local pid=$1
    local service_name=$2
    
    if [ -z "$pid" ]; then
        log_warning "$service_name PID 为空，跳过"
        return 0
    fi
    
    if kill -0 "$pid" 2>/dev/null; then
        log_info "停止 $service_name (PID: $pid)..."
        kill -TERM "$pid" 2>/dev/null
        
        # 等待进程优雅退出
        local count=0
        while kill -0 "$pid" 2>/dev/null && [ $count -lt 10 ]; do
            sleep 1
            count=$((count + 1))
        done
        
        # 如果进程仍然存在，强制杀死
        if kill -0 "$pid" 2>/dev/null; then
            log_warning "$service_name 未能优雅退出，强制终止..."
            kill -9 "$pid" 2>/dev/null
            sleep 1
        fi
        
        # 最终检查
        if kill -0 "$pid" 2>/dev/null; then
            log_error "无法停止 $service_name (PID: $pid)"
            return 1
        else
            log_success "$service_name 已停止"
            return 0
        fi
    else
        log_info "$service_name 未运行或已停止"
        return 0
    fi
}

# 从PID文件读取进程ID并停止
stop_from_pidfile() {
    local pidfile=$1
    local service_name=$2
    
    if [ -f "$pidfile" ]; then
        local pid=$(cat "$pidfile" 2>/dev/null)
        stop_process "$pid" "$service_name"
        rm -f "$pidfile"
    else
        log_info "未找到 $service_name 的PID文件"
    fi
}

# 按端口停止进程
stop_by_port() {
    local port=$1
    local service_name=$2
    
    local pids=$(lsof -ti:$port 2>/dev/null)
    
    if [ ! -z "$pids" ]; then
        log_info "发现端口 $port 上的进程: $pids"
        for pid in $pids; do
            stop_process "$pid" "$service_name (端口 $port)"
        done
    else
        log_info "端口 $port 上没有运行的进程"
    fi
}

# 创建logs目录
mkdir -p logs

log_info "正在查找并停止服务..."
echo ""

# 方法1: 从PID文件停止
log_info "从PID文件停止服务..."
stop_from_pidfile "logs/main_service.pid" "主服务"
stop_from_pidfile "logs/external_receiver.pid" "外部接收器"
stop_from_pidfile "logs/incoming_receiver.pid" "外部消息接收器"

echo ""

# 方法2: 按端口停止（确保清理）
log_info "检查并清理端口占用..."
stop_by_port "8080" "主服务"
stop_by_port "3000" "外部接收器"
stop_by_port "3001" "外部消息接收器"

echo ""

# 方法3: 按进程名停止（最后的保险）
log_info "检查相关Python进程..."

# 查找相关的Python进程
python_processes=$(ps aux | grep -E "(app\.py|external_receiver_example\.py|incoming_receiver\.py)" | grep -v grep | awk '{print $2}')

if [ ! -z "$python_processes" ]; then
    log_info "发现相关Python进程: $python_processes"
    for pid in $python_processes; do
        # 获取进程命令行
        if ps -p "$pid" > /dev/null 2>&1; then
            local cmd=$(ps -p "$pid" -o command= 2>/dev/null)
            if [[ "$cmd" == *"app.py"* ]]; then
                stop_process "$pid" "主服务进程"
            elif [[ "$cmd" == *"external_receiver_example.py"* ]]; then
                stop_process "$pid" "外部接收器进程"
            elif [[ "$cmd" == *"incoming_receiver.py"* ]]; then
                stop_process "$pid" "外部消息接收器进程"
            fi
        fi
    done
else
    log_info "未发现相关Python进程"
fi

echo ""

# 清理PID文件和临时文件
log_info "清理临时文件..."
rm -f logs/*.pid
log_success "PID文件已清理"

# 最终验证
log_info "最终验证服务状态..."
echo ""

# 检查端口是否已释放
check_port_status() {
    local port=$1
    local service_name=$2
    
    if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1; then
        log_warning "端口 $port ($service_name) 仍被占用"
        return 1
    else
        log_success "端口 $port ($service_name) 已释放"
        return 0
    fi
}

check_port_status "8080" "主服务"
check_port_status "3000" "外部接收器"
check_port_status "3001" "外部消息接收器"

echo ""

# 显示日志文件信息
if [ -f "logs/main_service.log" ] || [ -f "logs/external_receiver.log" ] || [ -f "logs/incoming_receiver.log" ]; then
    log_info "服务日志文件保留在 logs/ 目录中:"
    [ -f "logs/main_service.log" ] && echo "  • 主服务日志: logs/main_service.log"
    [ -f "logs/external_receiver.log" ] && echo "  • 外部接收器日志: logs/external_receiver.log"
    [ -f "logs/incoming_receiver.log" ] && echo "  • 外部消息接收器日志: logs/incoming_receiver.log"
    echo ""
    
    read -p "$(echo -e ${YELLOW}"是否要清理日志文件？(y/N): "${NC})" clean_logs
    if [[ $clean_logs =~ ^[Yy]$ ]]; then
        rm -f logs/*.log
        log_success "日志文件已清理"
    else
        log_info "日志文件已保留"
    fi
fi

echo ""
log_header "🎉 所有服务已停止！"
echo ""
log_info "如需重新启动服务，请运行: ./start_all.sh"
echo ""

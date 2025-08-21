#!/bin/bash

# Gemini 多模态聊天系统 - 一键启动所有服务

set -e  # 遇到错误立即退出

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

# 清屏和显示欢迎信息
clear
log_header "============================================================"
log_header "    🚀 Gemini 2.5 Flash-Lite 多模态聊天系统启动器"
log_header "============================================================"
echo ""

# 检查当前目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

log_info "当前工作目录: $SCRIPT_DIR"
echo ""

# 检查必要文件
log_info "检查必要文件..."
required_files=("app.py" "index.html" "script.js" "styles.css" "requirements.txt" "external_receiver_example.py")
missing_files=()

for file in "${required_files[@]}"; do
    if [ ! -f "$file" ]; then
        missing_files+=("$file")
    fi
done

if [ ${#missing_files[@]} -ne 0 ]; then
    log_error "缺少必要文件: ${missing_files[*]}"
    exit 1
fi

log_success "所有必要文件检查完成"
echo ""

# 检查Python环境
log_info "检查Python环境..."
if ! command -v python3 &> /dev/null; then
    log_error "Python3 未安装"
    exit 1
fi

python_version=$(python3 --version 2>&1)
log_success "Python版本: $python_version"

# 检查pip
if ! command -v pip &> /dev/null && ! command -v pip3 &> /dev/null; then
    log_error "pip 未安装"
    exit 1
fi

# 创建必要目录
log_info "创建必要目录..."
mkdir -p uploads
mkdir -p logs

# 检查并安装依赖
log_info "检查Python依赖..."
if [ -f "requirements.txt" ]; then
    # 检查是否需要安装依赖
    log_info "检测依赖包状态..."
    
    # 简单检查关键包是否存在
    python3 -c "import flask, google.generativeai, PIL" 2>/dev/null
    if [ $? -ne 0 ]; then
        log_warning "检测到缺少依赖包，正在安装..."
        pip3 install -r requirements.txt
        
        if [ $? -ne 0 ]; then
            log_error "依赖安装失败"
            exit 1
        fi
        log_success "依赖安装完成"
    else
        log_success "依赖包检查完成"
    fi
else
    log_warning "未找到 requirements.txt 文件"
fi

echo ""

# 检查端口占用
log_info "检查端口占用情况..."

MAIN_PORT=1024
EXTERNAL_PORT=3000

check_port() {
    local port=$1
    local service_name=$2
    
    if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1; then
        log_warning "端口 $port ($service_name) 已被占用"
        
        # 尝试杀死占用端口的进程
        local pid=$(lsof -ti:$port)
        if [ ! -z "$pid" ]; then
            log_info "尝试释放端口 $port (PID: $pid)..."
            kill -9 $pid 2>/dev/null || true
            sleep 1
            
            # 再次检查
            if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1; then
                log_error "无法释放端口 $port，请手动处理"
                return 1
            else
                log_success "端口 $port 已释放"
            fi
        fi
    else
        log_success "端口 $port ($service_name) 可用"
    fi
    return 0
}

check_port $MAIN_PORT "主服务"
check_port $EXTERNAL_PORT "外部接收器"

echo ""

# 显示服务配置
log_header "服务配置信息:"
echo -e "  • 主服务端口:     ${CYAN}$MAIN_PORT${NC}"
echo -e "  • 外部接收器端口: ${CYAN}$EXTERNAL_PORT${NC}"
echo -e "  • 主界面地址:     ${CYAN}http://localhost:$MAIN_PORT${NC}"
echo -e "  • 外部接收器:     ${CYAN}http://localhost:$EXTERNAL_PORT${NC}"
echo -e "  • 转发URL设置:    ${CYAN}http://localhost:$EXTERNAL_PORT/api/receive${NC}"
echo ""

# 询问是否启动外部接收器
read -p "$(echo -e ${YELLOW}"是否启动外部消息接收器？(Y/n): "${NC})" start_external
start_external=${start_external:-Y}

echo ""
log_header "开始启动服务..."
echo ""

# 创建日志文件
MAIN_LOG="logs/main_service.log"
EXTERNAL_LOG="logs/external_receiver.log"

# 启动主服务
log_info "启动主服务 (端口: $MAIN_PORT)..."
# 确保使用虚拟环境
source venv/bin/activate
nohup python3 app.py > "$MAIN_LOG" 2>&1 &
MAIN_PID=$!

# 等待主服务启动
sleep 3

# 检查主服务是否启动成功
if kill -0 $MAIN_PID 2>/dev/null; then
    log_success "主服务启动成功 (PID: $MAIN_PID)"
    echo "$MAIN_PID" > logs/main_service.pid
else
    log_error "主服务启动失败，请检查日志: $MAIN_LOG"
    exit 1
fi

# 启动外部接收器（如果用户选择）
if [[ $start_external =~ ^[Yy]$ ]]; then
    log_info "启动外部消息接收器 (端口: $EXTERNAL_PORT)..."
    # 确保使用虚拟环境
    source venv/bin/activate
    nohup python3 external_receiver_example.py > "$EXTERNAL_LOG" 2>&1 &
    EXTERNAL_PID=$!
    
    # 等待外部接收器启动
    sleep 2
    
    # 检查外部接收器是否启动成功
    if kill -0 $EXTERNAL_PID 2>/dev/null; then
        log_success "外部接收器启动成功 (PID: $EXTERNAL_PID)"
        echo "$EXTERNAL_PID" > logs/external_receiver.pid
    else
        log_error "外部接收器启动失败，请检查日志: $EXTERNAL_LOG"
    fi
fi

echo ""

# 等待服务完全就绪
log_info "等待服务就绪..."
sleep 3

# 测试服务可用性
log_info "测试服务连接..."

# 测试主服务
curl -s "http://localhost:$MAIN_PORT/api/test" > /dev/null
if [ $? -eq 0 ]; then
    log_success "主服务连接正常"
else
    log_warning "主服务连接测试失败，服务可能仍在启动中"
fi

# 测试外部接收器（如果启动了）
if [[ $start_external =~ ^[Yy]$ ]]; then
    curl -s "http://localhost:$EXTERNAL_PORT/api/health" > /dev/null
    if [ $? -eq 0 ]; then
        log_success "外部接收器连接正常"
    else
        log_warning "外部接收器连接测试失败，服务可能仍在启动中"
    fi
fi

echo ""

# 显示启动完成信息
log_header "🎉 所有服务启动完成！"
echo ""
log_header "📱 访问地址:"
echo -e "  • 主界面:       ${GREEN}http://localhost:$MAIN_PORT${NC}"
echo -e "  • 打开文件:     ${GREEN}$SCRIPT_DIR/index.html${NC}"

if [[ $start_external =~ ^[Yy]$ ]]; then
    echo -e "  • 外部接收器:   ${GREEN}http://localhost:$EXTERNAL_PORT${NC}"
    echo ""
    log_header "🔗 使用说明:"
    echo -e "  1. 在浏览器中打开: ${CYAN}$SCRIPT_DIR/index.html${NC}"
    echo -e "  2. 在配置面板中设置转发URL: ${CYAN}http://localhost:$EXTERNAL_PORT/api/receive${NC}"
    echo -e "  3. 开始与AI聊天，消息将自动转发到外部程序"
fi

echo ""
log_header "📋 管理命令:"
echo -e "  • 查看主服务日志:     ${CYAN}tail -f $MAIN_LOG${NC}"
if [[ $start_external =~ ^[Yy]$ ]]; then
    echo -e "  • 查看外部接收器日志: ${CYAN}tail -f $EXTERNAL_LOG${NC}"
fi
echo -e "  • 停止所有服务:       ${CYAN}./stop_all.sh${NC}"
echo -e "  • 重启所有服务:       ${CYAN}./restart_all.sh${NC}"

echo ""
log_header "📁 项目文件:"
echo -e "  • 主服务:       ${CYAN}app.py${NC}"
echo -e "  • 前端界面:     ${CYAN}index.html${NC}"
echo -e "  • 外部接收器:   ${CYAN}external_receiver_example.py${NC}"

echo ""

# 保存服务信息
cat > logs/services.info << EOF
# Gemini 多模态聊天系统服务信息
# 启动时间: $(date)

MAIN_PID=$MAIN_PID
MAIN_PORT=$MAIN_PORT
MAIN_LOG=$MAIN_LOG

EOF

if [[ $start_external =~ ^[Yy]$ ]]; then
    cat >> logs/services.info << EOF
EXTERNAL_PID=$EXTERNAL_PID
EXTERNAL_PORT=$EXTERNAL_PORT
EXTERNAL_LOG=$EXTERNAL_LOG
EOF
fi

# 自动打开浏览器（macOS）
if command -v open &> /dev/null; then
    log_info "正在打开浏览器..."
    sleep 2
    open "file://$SCRIPT_DIR/index.html"
fi

echo ""
log_success "系统启动完成！按 Ctrl+C 停止脚本监控（服务将继续在后台运行）"
echo ""

# 实时监控服务状态
log_info "开始监控服务状态... (按 Ctrl+C 退出监控)"
echo ""

monitor_services() {
    while true; do
        printf "\r"
        
        # 检查主服务
        if kill -0 $MAIN_PID 2>/dev/null; then
            printf "${GREEN}✓ 主服务运行中${NC}  "
        else
            printf "${RED}✗ 主服务已停止${NC}  "
        fi
        
        # 检查外部接收器
        if [[ $start_external =~ ^[Yy]$ ]]; then
            if kill -0 $EXTERNAL_PID 2>/dev/null; then
                printf "${GREEN}✓ 外部接收器运行中${NC}  "
            else
                printf "${RED}✗ 外部接收器已停止${NC}  "
            fi
        fi
        
        printf "[$(date '+%H:%M:%S')]"
        
        sleep 5
    done
}

# 设置Ctrl+C处理
trap 'echo -e "\n\n${YELLOW}监控已停止，服务继续在后台运行${NC}"; echo "使用 ./stop_all.sh 停止所有服务"; exit 0' INT

# 开始监控
monitor_services

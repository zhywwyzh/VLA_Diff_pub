#!/bin/bash

# Gemini 多模态聊天系统 - 服务测试脚本

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

log_header "============================================================"
log_header "    🧪 Gemini 多模态聊天系统服务测试"
log_header "============================================================"
echo ""

# 测试HTTP服务
test_http_service() {
    local url=$1
    local service_name=$2
    local timeout=${3:-5}
    
    log_info "测试 $service_name..."
    
    # 使用curl测试服务
    local response=$(curl -s -w "%{http_code}" -m $timeout "$url" -o /dev/null 2>/dev/null)
    
    if [ "$response" = "200" ]; then
        log_success "$service_name 服务正常 ✓"
        return 0
    elif [ -z "$response" ]; then
        log_error "$service_name 无响应 ✗"
        return 1
    else
        log_warning "$service_name 响应异常 (HTTP $response) ⚠"
        return 1
    fi
}

# 测试端口连通性
test_port() {
    local host=$1
    local port=$2
    local service_name=$3
    
    log_info "测试 $service_name 端口连通性..."
    
    if timeout 3 bash -c "</dev/tcp/$host/$port" 2>/dev/null; then
        log_success "$service_name 端口 $port 可访问 ✓"
        return 0
    else
        log_error "$service_name 端口 $port 不可访问 ✗"
        return 1
    fi
}

# 检查进程是否运行
check_process() {
    local process_name=$1
    local service_name=$2
    
    log_info "检查 $service_name 进程..."
    
    local pids=$(pgrep -f "$process_name" 2>/dev/null)
    
    if [ ! -z "$pids" ]; then
        log_success "$service_name 进程运行中 (PID: $pids) ✓"
        return 0
    else
        log_error "$service_name 进程未运行 ✗"
        return 1
    fi
}

# 测试API端点
test_api_endpoint() {
    local url=$1
    local endpoint_name=$2
    local expected_content=$3
    
    log_info "测试 $endpoint_name API..."
    
    local response=$(curl -s -m 5 "$url" 2>/dev/null)
    
    if [ $? -eq 0 ] && [ ! -z "$response" ]; then
        if [ ! -z "$expected_content" ] && [[ "$response" == *"$expected_content"* ]]; then
            log_success "$endpoint_name API 正常 ✓"
            return 0
        elif [ -z "$expected_content" ]; then
            log_success "$endpoint_name API 有响应 ✓"
            return 0
        else
            log_warning "$endpoint_name API 响应内容异常 ⚠"
            echo "  响应内容: ${response:0:100}..."
            return 1
        fi
    else
        log_error "$endpoint_name API 无响应 ✗"
        return 1
    fi
}

# 开始测试
echo "开始服务健康检查..."
echo ""

total_tests=0
passed_tests=0

# 测试主服务
log_header "📱 主服务测试 (端口 8080)"
echo ""

total_tests=$((total_tests + 1))
if check_process "app.py" "主服务"; then
    passed_tests=$((passed_tests + 1))
fi

total_tests=$((total_tests + 1))
if test_port "localhost" "8080" "主服务"; then
    passed_tests=$((passed_tests + 1))
fi

total_tests=$((total_tests + 1))
if test_api_endpoint "http://localhost:8080/api/test" "主服务健康检查" "success"; then
    passed_tests=$((passed_tests + 1))
fi

echo ""

# 测试外部接收器
log_header "🔄 外部接收器测试 (端口 3000)"
echo ""

total_tests=$((total_tests + 1))
if check_process "external_receiver_example.py" "外部接收器"; then
    passed_tests=$((passed_tests + 1))
    
    total_tests=$((total_tests + 1))
    if test_port "localhost" "3000" "外部接收器"; then
        passed_tests=$((passed_tests + 1))
    fi
    
    total_tests=$((total_tests + 1))
    if test_api_endpoint "http://localhost:3000/api/health" "外部接收器健康检查" "running"; then
        passed_tests=$((passed_tests + 1))
    fi
else
    log_info "外部接收器未启动，跳过相关测试"
fi

echo ""

# 测试文件完整性
log_header "📁 文件完整性测试"
echo ""

required_files=("app.py" "index.html" "script.js" "styles.css" "requirements.txt")
file_tests=0
file_passed=0

for file in "${required_files[@]}"; do
    file_tests=$((file_tests + 1))
    total_tests=$((total_tests + 1))
    
    if [ -f "$file" ]; then
        log_success "文件 $file 存在 ✓"
        file_passed=$((file_passed + 1))
        passed_tests=$((passed_tests + 1))
    else
        log_error "文件 $file 缺失 ✗"
    fi
done

echo ""

# 测试依赖包
log_header "📦 Python依赖测试"
echo ""

dependency_tests=0
dependency_passed=0

critical_packages=("flask" "google.generativeai" "PIL" "speech_recognition" "gtts")

for package in "${critical_packages[@]}"; do
    dependency_tests=$((dependency_tests + 1))
    total_tests=$((total_tests + 1))
    
    log_info "检查 $package 包..."
    
    if python3 -c "import $package" 2>/dev/null; then
        log_success "$package 包可用 ✓"
        dependency_passed=$((dependency_passed + 1))
        passed_tests=$((passed_tests + 1))
    else
        log_error "$package 包不可用 ✗"
    fi
done

echo ""

# 功能测试（如果服务运行中）
if curl -s "http://localhost:8080/api/test" > /dev/null 2>&1; then
    log_header "🧩 功能性测试"
    echo ""
    
    # 测试简单的聊天API
    total_tests=$((total_tests + 1))
    log_info "测试聊天API..."
    
    test_data='{"text":"测试消息","image":null,"audio":null}'
    response=$(curl -s -X POST -H "Content-Type: application/json" \
                   -d "$test_data" "http://localhost:8080/api/chat" 2>/dev/null)
    
    if [ $? -eq 0 ] && [[ "$response" == *"success"* ]]; then
        log_success "聊天API 功能正常 ✓"
        passed_tests=$((passed_tests + 1))
    else
        log_warning "聊天API 测试失败 ⚠"
        echo "  响应: ${response:0:100}..."
    fi
    
    echo ""
fi

# 显示测试结果
log_header "📊 测试结果汇总"
echo ""

echo -e "总测试数:     ${CYAN}$total_tests${NC}"
echo -e "通过测试:     ${GREEN}$passed_tests${NC}"
echo -e "失败测试:     ${RED}$((total_tests - passed_tests))${NC}"

# 计算通过率
if [ $total_tests -gt 0 ]; then
    pass_rate=$((passed_tests * 100 / total_tests))
    echo -e "通过率:       ${CYAN}$pass_rate%${NC}"
    
    if [ $pass_rate -ge 90 ]; then
        echo -e "状态:         ${GREEN}优秀 🎉${NC}"
    elif [ $pass_rate -ge 70 ]; then
        echo -e "状态:         ${YELLOW}良好 👍${NC}"
    elif [ $pass_rate -ge 50 ]; then
        echo -e "状态:         ${YELLOW}一般 ⚠${NC}"
    else
        echo -e "状态:         ${RED}需要检查 ❌${NC}"
    fi
else
    echo -e "状态:         ${RED}无法评估${NC}"
fi

echo ""

# 根据结果给出建议
if [ $passed_tests -lt $total_tests ]; then
    log_header "🔧 建议操作"
    echo ""
    
    if [ $passed_tests -eq 0 ]; then
        echo "• 检查服务是否已启动: ./start_all.sh"
        echo "• 检查Python依赖: pip install -r requirements.txt"
    elif [ $((total_tests - passed_tests)) -le 2 ]; then
        echo "• 大部分功能正常，检查失败项目的具体错误"
        echo "• 查看服务日志: tail -f logs/*.log"
    else
        echo "• 重启所有服务: ./restart_all.sh"
        echo "• 检查系统资源和网络连接"
        echo "• 查看详细日志排查问题"
    fi
    
    echo ""
fi

log_header "测试完成！"

# 退出码反映测试结果
if [ $passed_tests -eq $total_tests ]; then
    exit 0
else
    exit 1
fi

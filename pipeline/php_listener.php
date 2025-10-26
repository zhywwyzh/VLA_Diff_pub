<?php
/**
 * PHP 版本的文字监听器
 * 接收从Gemini聊天系统转发的文字内容
 */

header('Content-Type: application/json; charset=utf-8');
header('Access-Control-Allow-Origin: *');
header('Access-Control-Allow-Methods: POST, GET, OPTIONS');
header('Access-Control-Allow-Headers: Content-Type');

// 处理预检请求
if ($_SERVER['REQUEST_METHOD'] === 'OPTIONS') {
    http_response_code(200);
    exit;
}

/**
 * 处理接收到的文字内容
 */
function processText($text, $source = 'gemini-chat') {
    echo "\n🔥 开始处理文字内容...\n";
    echo "📝 内容: $text\n";
    echo "📊 长度: " . mb_strlen($text) . " 字符\n";
    echo "🕒 时间: " . date('Y-m-d H:i:s') . "\n";
    
    // ========================================
    // 在这里添加您的处理逻辑
    // ========================================
    
    // 示例1: 文字分析
    if (mb_strlen($text) > 50) {
        echo "📏 这是一条长消息\n";
    } else {
        echo "📏 这是一条短消息\n";
    }
    
    // 示例2: 关键词检测
    $keywords = ['测试', '帮助', '问题', 'API', 'PHP'];
    $foundKeywords = [];
    foreach ($keywords as $keyword) {
        if (strpos($text, $keyword) !== false) {
            $foundKeywords[] = $keyword;
        }
    }
    if (!empty($foundKeywords)) {
        echo "🔍 发现关键词: " . implode(', ', $foundKeywords) . "\n";
    }
    
    // 示例3: 保存到文件
    try {
        $logData = "[" . date('Y-m-d H:i:s') . "] $text\n";
        file_put_contents('php_received_texts.txt', $logData, FILE_APPEND | LOCK_EX);
        echo "💾 已保存到文件\n";
    } catch (Exception $e) {
        echo "❌ 保存失败: " . $e->getMessage() . "\n";
    }
    
    // 示例4: 发送到其他系统
    // sendToOtherSystem($text);
    
    echo "✅ 处理完成\n\n";
}

/**
 * 发送到其他系统的示例函数
 */
function sendToOtherSystem($text) {
    // 示例：使用cURL发送到其他API
    /*
    $ch = curl_init();
    curl_setopt($ch, CURLOPT_URL, 'http://your-other-system.com/api');
    curl_setopt($ch, CURLOPT_POST, 1);
    curl_setopt($ch, CURLOPT_POSTFIELDS, json_encode(['text' => $text]));
    curl_setopt($ch, CURLOPT_HTTPHEADER, ['Content-Type: application/json']);
    curl_setopt($ch, CURLOPT_RETURNTRANSFER, true);
    
    $response = curl_exec($ch);
    $httpCode = curl_getinfo($ch, CURLINFO_HTTP_CODE);
    curl_close($ch);
    
    if ($httpCode === 200) {
        echo "发送到其他系统成功\n";
    } else {
        echo "发送到其他系统失败: HTTP $httpCode\n";
    }
    */
}

/**
 * 保存消息到JSON文件
 */
function saveMessage($message) {
    $filename = 'php_messages.json';
    $messages = [];
    
    if (file_exists($filename)) {
        $content = file_get_contents($filename);
        $messages = json_decode($content, true) ?: [];
    }
    
    $messages[] = $message;
    file_put_contents($filename, json_encode($messages, JSON_PRETTY_PRINT | JSON_UNESCAPED_UNICODE));
}

// 路由处理
$requestUri = $_SERVER['REQUEST_URI'];
$requestMethod = $_SERVER['REQUEST_METHOD'];

if ($requestUri === '/api/receive' && $requestMethod === 'POST') {
    // 接收文字消息
    $input = file_get_contents('php://input');
    $data = json_decode($input, true);
    
    if (!$data || !isset($data['text']) || empty($data['text'])) {
        http_response_code(400);
        echo json_encode([
            'success' => false,
            'error' => '文字内容为空'
        ]);
        exit;
    }
    
    $text = $data['text'];
    $source = $data['source'] ?? 'unknown';
    $timestamp = $data['timestamp'] ?? date('c');
    
    // 创建消息记录
    $message = [
        'id' => time() . rand(1000, 9999),
        'text' => $text,
        'source' => $source,
        'timestamp' => $timestamp,
        'received_at' => date('c'),
        'length' => mb_strlen($text)
    ];
    
    // 保存消息
    saveMessage($message);
    
    // 处理文字（在后台运行，避免阻塞响应）
    if (function_exists('fastcgi_finish_request')) {
        // 如果是FastCGI环境，立即返回响应
        echo json_encode([
            'success' => true,
            'message' => '文字已接收并开始处理',
            'id' => $message['id'],
            'length' => $message['length']
        ]);
        fastcgi_finish_request();
        processText($text, $source);
    } else {
        // 普通环境，直接处理
        processText($text, $source);
        echo json_encode([
            'success' => true,
            'message' => '文字已接收并处理完成',
            'id' => $message['id'],
            'length' => $message['length']
        ]);
    }
    
} elseif ($requestUri === '/api/messages' && $requestMethod === 'GET') {
    // 获取所有消息
    $filename = 'php_messages.json';
    $messages = [];
    
    if (file_exists($filename)) {
        $content = file_get_contents($filename);
        $messages = json_decode($content, true) ?: [];
    }
    
    echo json_encode([
        'success' => true,
        'total' => count($messages),
        'messages' => $messages
    ]);
    
} elseif ($requestUri === '/api/health' && $requestMethod === 'GET') {
    // 健康检查
    $filename = 'php_messages.json';
    $messageCount = 0;
    
    if (file_exists($filename)) {
        $content = file_get_contents($filename);
        $messages = json_decode($content, true) ?: [];
        $messageCount = count($messages);
    }
    
    echo json_encode([
        'success' => true,
        'status' => 'running',
        'service' => 'php-text-listener',
        'message_count' => $messageCount,
        'time' => date('c')
    ]);
    
} elseif ($requestUri === '/' && $requestMethod === 'GET') {
    // 主页
    $filename = 'php_messages.json';
    $messageCount = 0;
    
    if (file_exists($filename)) {
        $content = file_get_contents($filename);
        $messages = json_decode($content, true) ?: [];
        $messageCount = count($messages);
    }
    
    header('Content-Type: text/html; charset=utf-8');
    echo "
    <h1>🎧 PHP 文字监听器</h1>
    <p><strong>状态:</strong> 运行中</p>
    <p><strong>接收端点:</strong> /api/receive</p>
    <p><strong>消息数量:</strong> $messageCount</p>
    <p><strong>时间:</strong> " . date('Y-m-d H:i:s') . "</p>
    <p><strong>说明:</strong> 这个PHP服务会接收并处理转发的文字内容</p>
    ";
    
} else {
    // 404
    http_response_code(404);
    echo json_encode([
        'success' => false,
        'error' => 'Not Found'
    ]);
}
?>

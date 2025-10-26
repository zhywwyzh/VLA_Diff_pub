#!/usr/bin/env node

/**
 * Node.js 版本的文字监听器
 * 接收从Gemini聊天系统转发的文字内容
 */

const express = require('express');
const fs = require('fs');
const path = require('path');

const app = express();
const PORT = 7000;

// 中间件
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// 存储接收到的消息
let receivedMessages = [];

/**
 * 处理接收到的文字内容
 * @param {string} text - 接收到的文字
 * @param {string} source - 消息来源
 */
function processText(text, source = 'gemini-chat') {
    console.log('\n🔥 开始处理文字内容...');
    console.log(`📝 内容: ${text}`);
    console.log(`📊 长度: ${text.length} 字符`);
    console.log(`🕒 时间: ${new Date().toLocaleString()}`);
    
    // ========================================
    // 在这里添加您的处理逻辑
    // ========================================
    
    // 示例1: 文字分析
    if (text.length > 50) {
        console.log('📏 这是一条长消息');
    } else {
        console.log('📏 这是一条短消息');
    }
    
    // 示例2: 关键词检测
    const keywords = ['测试', '帮助', '问题', 'API', 'Node.js'];
    const foundKeywords = keywords.filter(keyword => text.includes(keyword));
    if (foundKeywords.length > 0) {
        console.log(`🔍 发现关键词: ${foundKeywords.join(', ')}`);
    }
    
    // 示例3: 保存到文件
    try {
        const logData = `[${new Date().toISOString()}] ${text}\n`;
        fs.appendFileSync('nodejs_received_texts.txt', logData, 'utf8');
        console.log('💾 已保存到文件');
    } catch (error) {
        console.log(`❌ 保存失败: ${error.message}`);
    }
    
    // 示例4: 发送到其他系统
    // sendToOtherSystem(text);
    
    console.log('✅ 处理完成\n');
}

/**
 * 发送到其他系统的示例函数
 */
function sendToOtherSystem(text) {
    // 示例：发送到其他API
    // const axios = require('axios');
    // try {
    //     const response = await axios.post('http://your-other-system.com/api', {
    //         text: text
    //     });
    //     console.log(`发送到其他系统成功: ${response.status}`);
    // } catch (error) {
    //     console.log(`发送到其他系统失败: ${error.message}`);
    // }
}

// 接收文字的API端点
app.post('/api/receive', (req, res) => {
    try {
        const { text, source, timestamp } = req.body;
        
        if (!text) {
            return res.status(400).json({
                success: false,
                error: '文字内容为空'
            });
        }
        
        // 创建消息记录
        const message = {
            id: receivedMessages.length + 1,
            text: text,
            source: source || 'unknown',
            timestamp: timestamp || new Date().toISOString(),
            received_at: new Date().toISOString(),
            length: text.length
        };
        
        // 保存消息
        receivedMessages.push(message);
        
        // 异步处理文字，避免阻塞响应
        setImmediate(() => processText(text, source));
        
        res.json({
            success: true,
            message: '文字已接收并开始处理',
            id: message.id,
            length: text.length
        });
        
    } catch (error) {
        console.error(`❌ 接收文字时出错: ${error.message}`);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

// 获取所有消息
app.get('/api/messages', (req, res) => {
    res.json({
        success: true,
        total: receivedMessages.length,
        messages: receivedMessages
    });
});

// 健康检查
app.get('/api/health', (req, res) => {
    res.json({
        success: true,
        status: 'running',
        service: 'nodejs-text-listener',
        message_count: receivedMessages.length,
        time: new Date().toISOString()
    });
});

// 主页
app.get('/', (req, res) => {
    res.send(`
        <h1>🎧 Node.js 文字监听器</h1>
        <p><strong>状态:</strong> 运行中</p>
        <p><strong>接收端点:</strong> /api/receive</p>
        <p><strong>消息数量:</strong> ${receivedMessages.length}</p>
        <p><strong>时间:</strong> ${new Date().toLocaleString()}</p>
        <p><strong>说明:</strong> 这个Node.js服务会接收并处理转发的文字内容</p>
    `);
});

// 启动服务器
app.listen(PORT, '0.0.0.0', () => {
    console.log('🎧 启动Node.js文字监听器...');
    console.log(`📍 服务地址: http://localhost:${PORT}`);
    console.log(`📍 接收端点: http://localhost:${PORT}/api/receive`);
    console.log(`💡 在Gemini聊天系统中设置转发URL为: http://localhost:${PORT}/api/receive`);
    console.log('🔧 请在 processText() 函数中添加您的处理逻辑');
    console.log('-'.repeat(60));
});

// 优雅关闭
process.on('SIGINT', () => {
    console.log('\n🛑 正在关闭服务器...');
    process.exit(0);
});

module.exports = app;

// 全局变量
let mediaRecorder = null;
let recordedChunks = [];
let isRecording = false;
let currentImages = [];  // 改为数组支持多图片
let currentAudio = null;
let isConnected = false;
let currentModel = 'gemini-2.5-flash-lite';  // 当前模型
let incomingMessages = [];  // 外部消息
let incomingMessagesPollInterval = null;  // 轮询定时器

// API基础URL
const API_BASE_URL = 'http://localhost:1024';
const INCOMING_API_URL = 'http://localhost:3001';

// 页面加载完成后初始化
document.addEventListener('DOMContentLoaded', function() {
    initializeApp();
});

// 初始化应用
function initializeApp() {
    console.log('初始化 Gemini 多模态聊天应用...');
    
    // 测试服务器连接
    testServerConnection();
    
    // 加载转发配置
    loadForwardConfig();
    
    // 初始化外部消息功能
    initializeIncomingMessages();
    
    // 绑定回车键发送消息
    document.getElementById('text-input').addEventListener('keypress', function(e) {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            sendTextMessage();
        }
    });
    
    // 绑定转发URL输入框的回车键
    document.getElementById('forward-url').addEventListener('keypress', function(e) {
        if (e.key === 'Enter') {
            e.preventDefault();
            setForwardUrl();
        }
    });
    
    // 检查浏览器支持
    checkBrowserSupport();
    
    showToast('应用已初始化', 'success');
}

// 测试服务器连接
async function testServerConnection() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/test`);
        const result = await response.json();
        
        if (result.success) {
            console.log('服务器连接正常');
            isConnected = true;
            updateConnectionStatus(true);
            showToast('已连接到服务器', 'success');
        } else {
            throw new Error('服务器测试失败');
        }
    } catch (error) {
        console.error('服务器连接失败:', error);
        isConnected = false;
        updateConnectionStatus(false);
        showToast('连接失败，请检查服务器状态', 'warning');
    }
}

// 更新连接状态
function updateConnectionStatus(connected) {
    const icon = document.getElementById('connection-icon');
    const status = document.getElementById('connection-status');
    
    if (connected) {
        icon.className = 'material-icons connected';
        icon.textContent = 'cloud_done';
        status.textContent = '已连接';
    } else {
        icon.className = 'material-icons disconnected';
        icon.textContent = 'cloud_off';
        status.textContent = '未连接';
    }
}

// 检查浏览器支持
function checkBrowserSupport() {
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
        showToast('您的浏览器不支持录音功能', 'warning');
        document.querySelector('.voice-control button').disabled = true;
    }
    
    if (!window.FileReader) {
        showToast('您的浏览器不支持文件读取功能', 'warning');
        document.querySelector('.image-control button').disabled = true;
    }
}

// 发送文字消息（支持多模态组合和多图片）
async function sendTextMessage() {
    const textInput = document.getElementById('text-input');
    const text = textInput.value.trim();
    
    if (!text && !currentImages.length && !currentAudio) {
        showToast('请输入消息、上传图片或录制语音', 'warning');
        return;
    }
    
    // 构建显示消息
    let displayText = text;
    let isVoice = false;
    
    if (currentAudio && text) {
        displayText = `${text} [包含语音]`;
        isVoice = true;
    } else if (currentAudio && !text) {
        displayText = '[语音消息]';
        isVoice = true;
    }
    
    if (currentImages.length > 0) {
        if (displayText) {
            displayText += ` [包含${currentImages.length}张图片]`;
        } else {
            displayText = `[${currentImages.length}张图片]`;
        }
    }
    
    // 显示用户消息
    addMessage('user', displayText, currentImages, isVoice);
    
    // 清空输入
    textInput.value = '';
    
    // 准备发送数据
    const messageData = {
        text: text,
        images: currentImages.map(img => img.data),  // 发送图片数据数组
        audio: currentAudio,
        model: currentModel  // 包含当前选择的模型
    };
    
    // 清除预览
    if (currentImages.length > 0) {
        clearImagePreview();
    }
    if (currentAudio) {
        clearAudioPreview();
    }
    
    // 发送消息
    await sendMessage(messageData);
}

// 切换语音录制
async function toggleVoiceRecording() {
    if (!isRecording) {
        await startVoiceRecording();
    } else {
        await stopVoiceRecording();
    }
}

// 开始语音录制
async function startVoiceRecording() {
    try {
        const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
        
        recordedChunks = [];
        mediaRecorder = new MediaRecorder(stream, {
            mimeType: 'audio/webm;codecs=opus'
        });
        
        mediaRecorder.ondataavailable = function(event) {
            if (event.data.size > 0) {
                recordedChunks.push(event.data);
            }
        };
        
        mediaRecorder.onstop = async function() {
            const audioBlob = new Blob(recordedChunks, { type: 'audio/webm' });
            await processAudioRecording(audioBlob);
            
            // 停止所有音频轨道
            stream.getTracks().forEach(track => track.stop());
        };
        
        mediaRecorder.start();
        isRecording = true;
        
        // 更新UI
        updateRecordingUI(true);
        showToast('开始录音...', 'success');
        
    } catch (error) {
        console.error('录音失败:', error);
        showToast('录音失败: ' + error.message, 'error');
    }
}

// 停止语音录制
async function stopVoiceRecording() {
    if (mediaRecorder && isRecording) {
        mediaRecorder.stop();
        isRecording = false;
        updateRecordingUI(false);
        showToast('录音结束，正在处理...', 'success');
    }
}

// 处理录音（现在不立即发送，而是保存为待发送状态）
async function processAudioRecording(audioBlob) {
    try {
        // 转换为base64
        const audioBase64 = await blobToBase64(audioBlob);
        
        // 保存录音数据
        currentAudio = audioBase64;
        
        // 显示录音预览
        showAudioPreview(audioBlob);
        
        showToast('语音录制完成，可以添加文字或图片后一起发送', 'success');
        
    } catch (error) {
        console.error('音频处理失败:', error);
        showToast('音频处理失败: ' + error.message, 'error');
    }
}

// 更新录音UI
function updateRecordingUI(recording) {
    const voiceBtn = document.getElementById('voice-btn');
    const recordingStatus = document.getElementById('recording-status');
    
    if (recording) {
        voiceBtn.classList.add('recording');
        voiceBtn.innerHTML = '<i class="material-icons">stop</i>';
        recordingStatus.style.display = 'block';
    } else {
        voiceBtn.classList.remove('recording');
        voiceBtn.innerHTML = '<i class="material-icons">mic</i>';
        recordingStatus.style.display = 'none';
    }
}

// 处理图片上传（支持多图片）
function handleImageUpload(event) {
    const files = Array.from(event.target.files);
    if (!files.length) return;
    
    const validFiles = [];
    
    for (const file of files) {
        // 检查文件类型
        if (!file.type.startsWith('image/')) {
            showToast(`${file.name} 不是图片文件，已跳过`, 'warning');
            continue;
        }
        
        // 检查文件大小 (最大5MB)
        if (file.size > 5 * 1024 * 1024) {
            showToast(`${file.name} 文件过大，已跳过`, 'warning');
            continue;
        }
        
        // 检查总数量限制 (最多8张)
        if (currentImages.length + validFiles.length >= 8) {
            showToast('最多只能上传8张图片', 'warning');
            break;
        }
        
        validFiles.push(file);
    }
    
    if (validFiles.length === 0) {
        showToast('没有有效的图片文件', 'error');
        return;
    }
    
    // 批量处理图片
    validFiles.forEach(file => {
        const reader = new FileReader();
        reader.onload = function(e) {
            currentImages.push({
                data: e.target.result,
                name: file.name,
                size: file.size
            });
            
            updateImagePreview();
            
            if (currentImages.length === 1) {
                showToast(`已上传 ${validFiles.length} 张图片`, 'success');
            }
        };
        reader.readAsDataURL(file);
    });
    
    // 清空file input
    event.target.value = '';
}

// 更新图片预览（支持多图片）
function updateImagePreview() {
    const preview = document.getElementById('image-preview');
    
    if (currentImages.length === 0) {
        preview.innerHTML = '';
        return;
    }
    
    let html = `<div class="image-count">已选择 ${currentImages.length} 张图片</div>`;
    html += '<div class="images-container">';
    
    currentImages.forEach((imageObj, index) => {
        html += `
            <div class="image-item">
                <img src="${imageObj.data}" alt="${imageObj.name}" />
                <button class="remove-single" onclick="removeImage(${index})" title="移除此图片">
                    ×
                </button>
            </div>
        `;
    });
    
    html += '</div>';
    html += '<button class="remove-all" onclick="clearImagePreview()">移除所有图片</button>';
    
    preview.innerHTML = html;
}

// 移除单张图片
function removeImage(index) {
    currentImages.splice(index, 1);
    updateImagePreview();
    showToast('图片已移除', 'success');
}

// 显示音频预览
function showAudioPreview(audioBlob) {
    const preview = document.getElementById('audio-preview') || createAudioPreviewElement();
    
    // 创建音频URL
    const audioUrl = URL.createObjectURL(audioBlob);
    
    preview.innerHTML = `
        <div class="audio-preview-content">
            <div class="audio-info">
                <i class="material-icons">mic</i>
                <span>录音已准备 (${(audioBlob.size / 1024).toFixed(1)} KB)</span>
            </div>
            <div class="audio-controls">
                <button onclick="playPreviewAudio('${audioUrl}')" class="play-btn">
                    <i class="material-icons">play_arrow</i> 试听
                </button>
                <button onclick="clearAudioPreview()" class="remove-btn">
                    <i class="material-icons">close</i> 移除
                </button>
            </div>
        </div>
    `;
    preview.style.display = 'block';
}

// 创建音频预览元素
function createAudioPreviewElement() {
    const preview = document.createElement('div');
    preview.id = 'audio-preview';
    preview.className = 'audio-preview';
    preview.style.display = 'none';
    
    // 插入到图片预览后面
    const imagePreview = document.getElementById('image-preview');
    imagePreview.parentNode.insertBefore(preview, imagePreview.nextSibling);
    
    return preview;
}

// 播放预览音频
function playPreviewAudio(audioUrl) {
    const audio = new Audio(audioUrl);
    audio.play().catch(error => {
        console.error('音频播放失败:', error);
        showToast('音频播放失败', 'warning');
    });
}

// 清除音频预览
function clearAudioPreview() {
    currentAudio = null;
    const preview = document.getElementById('audio-preview');
    if (preview) {
        preview.style.display = 'none';
        preview.innerHTML = '';
    }
}

// 清除图片预览
function clearImagePreview() {
    currentImages = [];
    document.getElementById('image-preview').innerHTML = '';
}

// 发送消息到服务器
async function sendMessage(messageData) {
    showLoading(true);
    
    try {
        // 设置较长的超时时间
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 60000); // 60秒超时
        
        const response = await fetch(`${API_BASE_URL}/api/chat`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(messageData),
            signal: controller.signal
        });
        
        clearTimeout(timeoutId);
        
        if (!response.ok) {
            throw new Error(`HTTP错误: ${response.status} ${response.statusText}`);
        }
        
        const result = await response.json();
        
        if (result.success) {
            // 显示助手回复
            addMessage('assistant', result.response.text);
            
            // 播放语音回复（如果有且不是错误消息）
            if (result.response.audio && !result.response.text.includes('抱歉') && !result.response.text.includes('错误')) {
                playAudio(result.response.audio);
            }
            
            // 显示语音识别结果
            if (result.response.recognized_speech) {
                if (result.response.recognized_speech.startsWith('[')) {
                    // 如果是错误信息，显示为警告
                    showToast(result.response.recognized_speech, 'warning');
                } else {
                    showToast(`语音识别: ${result.response.recognized_speech}`, 'success');
                }
            }
            
        } else {
            throw new Error(result.error || '发送消息失败');
        }
        
    } catch (error) {
        console.error('发送消息失败:', error);
        
        let errorMessage = '发送消息失败';
        let toastType = 'error';
        
        if (error.name === 'AbortError') {
            errorMessage = '请求超时，请检查网络连接或稍后重试';
            toastType = 'warning';
        } else if (error.message.includes('网络')) {
            errorMessage = '网络连接问题，请检查网络后重试';
            toastType = 'warning';
        } else if (error.message.includes('timeout') || error.message.includes('超时')) {
            errorMessage = '服务器响应超时，请稍后重试';
            toastType = 'warning';
        } else if (error.message.includes('503') || error.message.includes('502')) {
            errorMessage = '服务暂时不可用，请稍后重试';
            toastType = 'warning';
        }
        
        showToast(errorMessage, toastType);
        
        // 显示友好的错误消息
        if (error.message.includes('网络') || error.message.includes('timeout') || error.message.includes('503')) {
            addMessage('assistant', '🔄 抱歉，网络连接有问题或服务暂时不可用。请检查网络连接后重试。如果问题持续，可能是API服务暂时无法访问。');
        } else {
            addMessage('assistant', '😅 抱歉，处理您的消息时出现了问题。请稍后重试。');
        }
    } finally {
        showLoading(false);
    }
}

// 添加消息到聊天界面（支持多图片）
function addMessage(sender, text, images = null, isVoice = false) {
    const chatMessages = document.getElementById('chat-messages');
    
    // 移除欢迎消息
    const welcomeMessage = chatMessages.querySelector('.welcome-message');
    if (welcomeMessage) {
        welcomeMessage.remove();
    }
    
    const messageDiv = document.createElement('div');
    messageDiv.className = `message ${sender}`;
    
    let messageContent = '';
    
    // 添加图片（支持多图片）
    if (images) {
        if (Array.isArray(images) && images.length > 0) {
            // 多图片显示
            messageContent += '<div class="image-content">';
            images.forEach((imageObj, index) => {
                const imageSrc = typeof imageObj === 'string' ? imageObj : imageObj.data;
                messageContent += `<img src="${imageSrc}" alt="上传的图片${index + 1}" style="margin: 2px; max-width: 150px; max-height: 120px;" />`;
            });
            messageContent += '</div>';
        } else if (typeof images === 'string') {
            // 单图片兼容
            messageContent += `<div class="image-content"><img src="${images}" alt="上传的图片" /></div>`;
        }
    }
    
    // 添加文字内容
    if (text) {
        messageContent += `<div class="message-content">${text}</div>`;
    }
    
    // 添加元信息
    const timestamp = new Date().toLocaleTimeString();
    const voiceIcon = isVoice ? '<i class="material-icons">mic</i>' : '';
    messageContent += `<div class="message-meta">${voiceIcon} ${timestamp}</div>`;
    
    // 如果是助手消息，添加控制按钮
    if (sender === 'assistant') {
        messageContent += `
            <div class="message-controls">
                <button onclick="playTextAsVoice('${text.replace(/'/g, "\\'")}')">
                    <i class="material-icons">volume_up</i>
                </button>
                <button onclick="copyText('${text.replace(/'/g, "\\'")}')">
                    <i class="material-icons">content_copy</i>
                </button>
            </div>
        `;
    }
    
    messageDiv.innerHTML = messageContent;
    chatMessages.appendChild(messageDiv);
    
    // 滚动到底部
    chatMessages.scrollTop = chatMessages.scrollHeight;
}

// 播放音频
function playAudio(audioBase64) {
    try {
        const audioPlayer = document.getElementById('audio-player');
        audioPlayer.src = `data:audio/mp3;base64,${audioBase64}`;
        audioPlayer.play().catch(error => {
            console.error('音频播放失败:', error);
            showToast('音频播放失败', 'warning');
        });
    } catch (error) {
        console.error('音频处理失败:', error);
    }
}

// 朗读文本
async function playTextAsVoice(text) {
    if ('speechSynthesis' in window) {
        const utterance = new SpeechSynthesisUtterance(text);
        utterance.lang = 'zh-CN';
        speechSynthesis.speak(utterance);
    } else {
        showToast('您的浏览器不支持语音合成', 'warning');
    }
}

// 复制文本
function copyText(text) {
    navigator.clipboard.writeText(text).then(() => {
        showToast('文本已复制到剪贴板', 'success');
    }).catch(() => {
        showToast('复制失败', 'error');
    });
}

// 清除聊天历史
async function clearHistory() {
    if (!confirm('确定要清除所有聊天记录吗？')) {
        return;
    }
    
    try {
        const response = await fetch(`${API_BASE_URL}/api/clear`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            }
        });
        
        const result = await response.json();
        
        if (result.success) {
            // 清空聊天界面
            const chatMessages = document.getElementById('chat-messages');
            chatMessages.innerHTML = `
                <div class="welcome-message">
                    <i class="material-icons">waving_hand</i>
                    <p>欢迎使用 Gemini 多模态聊天助手！</p>
                    <p>您可以通过文字、语音或图片与我互动。</p>
                </div>
            `;
            showToast('聊天记录已清除', 'success');
        } else {
            throw new Error(result.error || '清除失败');
        }
        
    } catch (error) {
        console.error('清除历史失败:', error);
        showToast('清除失败: ' + error.message, 'error');
    }
}



// 显示加载状态
function showLoading(show) {
    const loadingOverlay = document.getElementById('loading-overlay');
    loadingOverlay.style.display = show ? 'flex' : 'none';
    
    // 禁用/启用输入
    const textInput = document.getElementById('text-input');
    const sendBtn = document.getElementById('send-btn');
    textInput.disabled = show;
    sendBtn.disabled = show;
}

// 显示Toast通知
function showToast(message, type = 'info') {
    const toastContainer = document.getElementById('toast-container');
    
    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    toast.textContent = message;
    
    toastContainer.appendChild(toast);
    
    // 3秒后自动移除
    setTimeout(() => {
        if (toast.parentNode) {
            toast.parentNode.removeChild(toast);
        }
    }, 3000);
}

// 工具函数：将Blob转换为base64
function blobToBase64(blob) {
    return new Promise((resolve, reject) => {
        const reader = new FileReader();
        reader.onload = () => {
            const result = reader.result;
            const base64 = result.split(',')[1]; // 移除data:audio/webm;base64,前缀
            resolve(base64);
        };
        reader.onerror = reject;
        reader.readAsDataURL(blob);
    });
}

// 错误处理
window.addEventListener('error', function(e) {
    console.error('全局错误:', e.error);
    showToast('发生了未知错误', 'error');
});

// 加载转发配置
async function loadForwardConfig() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/config/forward`);
        const result = await response.json();
        
        if (result.success && result.forward_url) {
            document.getElementById('forward-url').value = result.forward_url;
            updateForwardStatus('已设置转发URL', 'success');
        }
    } catch (error) {
        console.error('加载转发配置失败:', error);
    }
}

// 设置转发URL
async function setForwardUrl() {
    const urlInput = document.getElementById('forward-url');
    const url = urlInput.value.trim();
    
    try {
        const response = await fetch(`${API_BASE_URL}/api/config/forward`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ url: url })
        });
        
        const result = await response.json();
        
        if (result.success) {
            updateForwardStatus(result.message, 'success');
            if (result.connection_status) {
                updateForwardStatus(`${result.message} - ${result.connection_status}`, 
                    result.connection_status.includes('成功') ? 'success' : 'warning');
            }
            showToast(result.message, 'success');
        } else {
            updateForwardStatus(result.error, 'error');
            showToast(result.error, 'error');
        }
    } catch (error) {
        const errorMsg = '设置转发URL失败: ' + error.message;
        updateForwardStatus(errorMsg, 'error');
        showToast(errorMsg, 'error');
    }
}

// 更新转发状态显示
function updateForwardStatus(message, type) {
    const statusElement = document.getElementById('forward-status');
    statusElement.textContent = message;
    statusElement.className = `config-status ${type}`;
}

// 打开监听窗口
function openMonitorWindow() {
    // 打开本地监听窗口
    const monitorWindow = window.open(
        './message_monitor.html',
        'MessageMonitor',
        'width=1200,height=800,scrollbars=yes,resizable=yes,menubar=no,toolbar=no,location=no,status=no'
    );
    
    if (monitorWindow) {
        showToast('监听窗口已打开', 'success');
        // 窗口关闭时的处理
        monitorWindow.addEventListener('beforeunload', function() {
            showToast('监听窗口已关闭', 'info');
        });
    } else {
        showToast('无法打开监听窗口，请检查浏览器弹窗拦截设置', 'warning');
    }
}

// 发送指令到ROS2
async function sendToROS() {
    const commandInput = document.getElementById('ros-command');
    const command = commandInput.value.trim();
    
    if (!command) {
        showToast('请输入要发送的指令内容', 'warning');
        return;
    }
    
    try {
        updateROSStatus('正在发送指令...', 'info');
        
        const response = await fetch(`${API_BASE_URL}/api/ros/command`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ command: command })
        });
        
        const result = await response.json();
        
        if (result.success) {
            updateROSStatus('指令发送成功', 'success');
            showToast('指令已发送到ROS2节点', 'success');
            // 清空输入框
            commandInput.value = '';
        } else {
            updateROSStatus(result.error || '发送失败', 'error');
            showToast(result.error || '发送指令失败', 'error');
        }
    } catch (error) {
        const errorMsg = '发送指令失败: ' + error.message;
        updateROSStatus(errorMsg, 'error');
        showToast(errorMsg, 'error');
    }
}

// 填入最后的AI回复
function fillFromLastMessage() {
    const chatMessages = document.getElementById('chat-messages');
    const lastAssistantMessage = Array.from(chatMessages.querySelectorAll('.message.assistant'))
        .pop(); // 获取最后一个助手消息
    
    if (lastAssistantMessage) {
        const messageContent = lastAssistantMessage.querySelector('.message-content');
        if (messageContent) {
            document.getElementById('ros-command').value = messageContent.textContent.trim();
            showToast('已填入最后的AI回复', 'success');
        }
    } else {
        showToast('没有找到AI回复消息', 'warning');
    }
}

// 更新ROS状态显示
function updateROSStatus(message, type) {
    const statusElement = document.getElementById('ros-status');
    statusElement.textContent = message;
    statusElement.className = `config-status ${type}`;
}

// 切换模型
async function changeModel() {
    const modelSelect = document.getElementById('model-select');
    const selectedModel = modelSelect.value;
    
    try {
        const response = await fetch(`${API_BASE_URL}/api/config/model`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ model: selectedModel })
        });
        
        const result = await response.json();
        
        if (result.success) {
            currentModel = selectedModel;
            showToast(`模型已切换到: ${selectedModel}`, 'success');
        } else {
            showToast(`模型切换失败: ${result.error}`, 'error');
            // 恢复到之前的选择
            modelSelect.value = currentModel;
        }
    } catch (error) {
        showToast(`模型切换失败: ${error.message}`, 'error');
        // 恢复到之前的选择
        modelSelect.value = currentModel;
    }
}

// 初始化外部消息功能
function initializeIncomingMessages() {
    console.log('初始化外部消息功能...');
    
    // 立即加载一次
    refreshIncomingMessages();
    
    // 开始轮询
    startIncomingMessagesPoll();
}

// 开始轮询外部消息
function startIncomingMessagesPoll() {
    if (incomingMessagesPollInterval) {
        clearInterval(incomingMessagesPollInterval);
    }
    
    incomingMessagesPollInterval = setInterval(() => {
        refreshIncomingMessages();
    }, 3000); // 每3秒刷新一次
}

// 停止轮询
function stopIncomingMessagesPoll() {
    if (incomingMessagesPollInterval) {
        clearInterval(incomingMessagesPollInterval);
        incomingMessagesPollInterval = null;
    }
}

// 刷新外部消息
async function refreshIncomingMessages() {
    try {
        const response = await fetch(`${INCOMING_API_URL}/api/messages?per_page=20`, {
            timeout: 5000
        });
        
        if (response.ok) {
            const result = await response.json();
            if (result.success) {
                incomingMessages = result.messages || [];
                updateIncomingMessagesUI(result);
            }
        }
    } catch (error) {
        // 静默处理错误，避免频繁提示
        console.log('外部消息服务未连接:', error.message);
    }
}

// 更新外部消息UI
function updateIncomingMessagesUI(data) {
    const container = document.getElementById('incoming-messages');
    const messageCount = document.getElementById('message-count');
    const unreadCount = document.getElementById('unread-count');
    
    // 更新统计信息
    messageCount.textContent = `${data.total || 0} 条消息`;
    unreadCount.textContent = `${data.unread_count || 0} 未读`;
    unreadCount.style.display = (data.unread_count || 0) > 0 ? 'inline' : 'none';
    
    // 更新消息列表
    if (!incomingMessages.length) {
        container.innerHTML = `
            <div class="no-messages">
                <i class="material-icons">inbox</i>
                <p>暂无外部消息</p>
                <small>其他程序可以通过 3001 端口发送消息</small>
            </div>
        `;
        return;
    }
    
    let html = '';
    incomingMessages.forEach(msg => {
        const isUnread = !msg.read;
        const priorityClass = msg.priority !== 'normal' ? `priority-${msg.priority}` : '';
        const timeStr = new Date(msg.timestamp).toLocaleString();
        
        html += `
            <div class="incoming-message ${isUnread ? 'unread' : ''} ${priorityClass}" 
                 onclick="selectIncomingMessage(${msg.id})">
                <div class="message-header">
                    <span class="message-sender">${msg.sender || 'unknown'}</span>
                    <span class="message-time">${timeStr}</span>
                </div>
                <div class="message-content">${msg.text}</div>
                <div class="message-actions">
                    <button class="action-btn transfer-btn" onclick="transferToInput(${msg.id}); event.stopPropagation();">
                        转入对话框
                    </button>
                    ${isUnread ? `<button class="action-btn mark-read-btn-small" onclick="markMessageRead(${msg.id}); event.stopPropagation();">
                        标记已读
                    </button>` : ''}
                </div>
            </div>
        `;
    });
    
    container.innerHTML = html;
}

// 选择外部消息
function selectIncomingMessage(messageId) {
    const message = incomingMessages.find(msg => msg.id === messageId);
    if (message && !message.read) {
        markMessageRead(messageId);
    }
}

// 转入对话框
function transferToInput(messageId) {
    const message = incomingMessages.find(msg => msg.id === messageId);
    if (message) {
        const textInput = document.getElementById('text-input');
        textInput.value = message.text;
        textInput.focus();
        
        // 标记为已读
        if (!message.read) {
            markMessageRead(messageId);
        }
        
        showToast('消息已转入对话框', 'success');
    }
}

// 标记消息为已读
async function markMessageRead(messageId) {
    try {
        const response = await fetch(`${INCOMING_API_URL}/api/messages/${messageId}/read`, {
            method: 'POST'
        });
        
        if (response.ok) {
            // 更新本地状态
            const message = incomingMessages.find(msg => msg.id === messageId);
            if (message) {
                message.read = true;
            }
            
            // 立即刷新UI
            const data = {
                messages: incomingMessages,
                total: incomingMessages.length,
                unread_count: incomingMessages.filter(msg => !msg.read).length
            };
            updateIncomingMessagesUI(data);
        }
    } catch (error) {
        console.error('标记已读失败:', error);
    }
}

// 标记所有消息为已读
async function markAllRead() {
    try {
        const response = await fetch(`${INCOMING_API_URL}/api/messages/mark_all_read`, {
            method: 'POST'
        });
        
        if (response.ok) {
            // 更新本地状态
            incomingMessages.forEach(msg => msg.read = true);
            
            // 立即刷新UI
            const data = {
                messages: incomingMessages,
                total: incomingMessages.length,
                unread_count: 0
            };
            updateIncomingMessagesUI(data);
            
            showToast('所有消息已标记为已读', 'success');
        }
    } catch (error) {
        showToast('操作失败', 'error');
    }
}

// 清空外部消息
async function clearIncomingMessages() {
    if (!confirm('确定要清空所有外部消息吗？')) {
        return;
    }
    
    try {
        const response = await fetch(`${INCOMING_API_URL}/api/clear`, {
            method: 'POST'
        });
        
        if (response.ok) {
            incomingMessages = [];
            const data = {
                messages: [],
                total: 0,
                unread_count: 0
            };
            updateIncomingMessagesUI(data);
            
            showToast('外部消息已清空', 'success');
        }
    } catch (error) {
        showToast('清空失败', 'error');
    }
}

// 打开消息发送器页面
function openMessageSender() {
    const senderUrl = 'message_sender.html';
    const windowFeatures = 'width=600,height=700,scrollbars=yes,resizable=yes,location=no,menubar=no,toolbar=no';
    
    const popup = window.open(senderUrl, 'MessageSender', windowFeatures);
    
    if (popup) {
        popup.focus();
        showToast('已打开消息发送器', 'success');
    } else {
        showToast('无法打开弹窗，请检查浏览器设置', 'error');
    }
}

// 页面卸载时清理资源
window.addEventListener('beforeunload', function() {
    if (mediaRecorder && isRecording) {
        mediaRecorder.stop();
    }
    
    // 停止轮询
    stopIncomingMessagesPoll();
});

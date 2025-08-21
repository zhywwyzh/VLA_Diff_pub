// 全局变量
let mediaRecorder = null;
let recordedChunks = [];
let isRecording = false;
let currentImage = null;
let currentAudio = null;
let isConnected = false;

// API基础URL
const API_BASE_URL = 'http://localhost:1024';

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

// 发送文字消息（现在支持多模态组合）
async function sendTextMessage() {
    const textInput = document.getElementById('text-input');
    const text = textInput.value.trim();
    
    if (!text && !currentImage && !currentAudio) {
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
    
    // 显示用户消息
    addMessage('user', displayText, currentImage, isVoice);
    
    // 清空输入
    textInput.value = '';
    
    // 准备发送数据
    const messageData = {
        text: text,
        image: currentImage,
        audio: currentAudio
    };
    
    // 清除预览
    if (currentImage) {
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

// 处理图片上传
function handleImageUpload(event) {
    const file = event.target.files[0];
    if (!file) return;
    
    // 检查文件类型
    if (!file.type.startsWith('image/')) {
        showToast('请选择图片文件', 'error');
        return;
    }
    
    // 检查文件大小 (最大5MB)
    if (file.size > 5 * 1024 * 1024) {
        showToast('图片文件过大，请选择小于5MB的图片', 'error');
        return;
    }
    
    const reader = new FileReader();
    reader.onload = function(e) {
        currentImage = e.target.result;
        showImagePreview(currentImage);
        showToast('图片已上传', 'success');
    };
    reader.readAsDataURL(file);
    
    // 清空file input
    event.target.value = '';
}

// 显示图片预览
function showImagePreview(imageSrc) {
    const preview = document.getElementById('image-preview');
    preview.innerHTML = `
        <img src="${imageSrc}" alt="预览图片" />
        <button class="remove-image" onclick="clearImagePreview()">
            <i class="material-icons">close</i> 移除图片
        </button>
    `;
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
    currentImage = null;
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

// 添加消息到聊天界面
function addMessage(sender, text, image = null, isVoice = false) {
    const chatMessages = document.getElementById('chat-messages');
    
    // 移除欢迎消息
    const welcomeMessage = chatMessages.querySelector('.welcome-message');
    if (welcomeMessage) {
        welcomeMessage.remove();
    }
    
    const messageDiv = document.createElement('div');
    messageDiv.className = `message ${sender}`;
    
    let messageContent = '';
    
    // 添加图片
    if (image) {
        messageContent += `<div class="image-content"><img src="${image}" alt="上传的图片" /></div>`;
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
    const forwardUrl = document.getElementById('forward-url').value.trim();
    
    if (!forwardUrl) {
        showToast('请先设置转发URL', 'warning');
        return;
    }
    
    // 提取基础URL
    let baseUrl;
    try {
        const url = new URL(forwardUrl);
        baseUrl = `${url.protocol}//${url.host}`;
    } catch (error) {
        showToast('转发URL格式不正确', 'error');
        return;
    }
    
    // 打开监听窗口
    const monitorWindow = window.open(
        baseUrl,
        'MessageMonitor',
        'width=800,height=600,scrollbars=yes,resizable=yes'
    );
    
    if (monitorWindow) {
        showToast('监听窗口已打开', 'success');
    } else {
        showToast('无法打开监听窗口，请检查浏览器弹窗拦截设置', 'warning');
    }
}

// 页面卸载时清理资源
window.addEventListener('beforeunload', function() {
    if (mediaRecorder && isRecording) {
        mediaRecorder.stop();
    }
});

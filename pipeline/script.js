// 全局变量
let mediaRecorder = null;
let recordedChunks = [];
let isRecording = false;
let currentImages = [];  // 改为数组支持多图片
let selectedImages = []; // 当前选择的图片（用于图片预览）
let currentAudio = null;
let isConnected = false;
let currentModel = 'gemini-2.5-flash-lite';  // 当前模型
let incomingMessages = [];  // 外部消息
let incomingMessagesPollInterval = null;  // 轮询定时器

// 实时模型相关变量
let realtimePollingInterval = null;
let lastResponseTimestamp = 0;
let isRealtimePolling = false;

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
    
    // 加载可用模型并同步当前选择
    loadAvailableModels();
    
    // 同步当前模型状态
    syncCurrentModel();
    
    // 加载转发配置
    loadForwardConfig();
    
    // 初始化外部消息功能
    initializeIncomingMessages();
    
    // 加载语速设置
    loadSpeechRate();
    
    // 加载豆包深度思考设置
    loadDoubaoThinkingConfig();
    
    // 初始化深度思考控制可见性
    updateThinkingControlVisibility(currentModel);
    
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
    
    // 调试日志：记录发送的数据
    console.log('🔍 发送数据调试:', {
        model: currentModel,
        textLength: text ? text.length : 0,
        imageCount: currentImages.length,
        hasAudio: !!currentAudio
    });
    
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
    
    // 确保currentImages和selectedImages同步
    if (selectedImages.length > 0 && currentImages.length !== selectedImages.length) {
        // 如果selectedImages有内容但currentImages没有，同步过去
        currentImages = selectedImages.map(img => ({
            data: img.data,
            type: 'image/jpeg',
            name: img.name
        }));
    }
    
    if (currentImages.length === 0) {
        preview.innerHTML = '';
        return;
    }
    
    let html = `<div class="image-count">已选择 ${currentImages.length} 张图片</div>`;
    html += '<div class="images-container">';
    
    currentImages.forEach((imageObj, index) => {
        const imageName = imageObj.name || `图片${index + 1}`;
        html += `
            <div class="image-item">
                <img src="${imageObj.data}" alt="${imageName}" title="${imageName}" />
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
    // 同时从selectedImages中移除对应的图片
    if (selectedImages.length > index) {
        selectedImages.splice(index, 1);
    }
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
    selectedImages = []; // 同时清空selectedImages
    document.getElementById('image-preview').innerHTML = '';
}

// 发送消息到服务器
async function sendMessage(messageData) {
    showLoading(true);
    
    // 检查是否使用实时模型
    if (isRealtimeModel(currentModel)) {
        // 实时模型使用专门的发送方式
        showLoading(false);
        await sendRealtimeMessage(messageData);
        return;
    }
    
    try {
        // 设置较长的超时时间
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 180000); // 180秒超时，适应复杂prompt
        
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
            // 显示助手回复，传递音频数据以便后续重播
            addMessage('assistant', result.response.text, null, false, result.response.audio);
            
            // 自动播放ChatGLM生成的语音（如果有）
            if (result.response.audio && result.response.audio !== "chatglm_placeholder") {
                playAudio(result.response.audio, 'wav');
                showToast('正在播放ChatGLM语音', 'success');
            } else if (!result.response.text.includes('抱歉') && !result.response.text.includes('错误')) {
                // 如果没有预生成的语音，使用备用ChatGLM API
                playTextAsVoice(result.response.text);
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

// 添加消息到聊天界面（支持多图片和音频重播）
function addMessage(sender, text, images = null, isVoice = false, audioData = null) {
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
    const audioIcon = (audioData && audioData !== "chatglm_placeholder") ? '<i class="material-icons">volume_up</i>' : '';
    messageContent += `<div class="message-meta">${voiceIcon}${audioIcon} ${timestamp}</div>`;
    
    // 添加消息控制按钮
    const messageId = Date.now() + Math.random(); // 生成唯一ID
    messageDiv.setAttribute('data-message-id', messageId);
    
    // 如果有音频数据，保存到元素上
    if (audioData && audioData !== "chatglm_placeholder") {
        messageDiv.setAttribute('data-audio', audioData);
    }
    
    const controlsHtml = [];
    
    // 保存为指令记忆按钮（所有消息都可以保存）
    controlsHtml.push(`
        <button onclick="saveAsMemory('${messageId}', '${sender}')" title="保存为指令记忆">
            <i class="material-icons">bookmark_add</i>
        </button>
    `);
    
    // 助手消息的额外控制
    if (sender === 'assistant') {
        // 如果有ChatGLM生成的音频，优先使用重播按钮
        if (audioData && audioData !== "chatglm_placeholder") {
            controlsHtml.push(`
                <button onclick="replayMessageAudio('${messageId}')" title="重播ChatGLM语音">
                    <i class="material-icons">replay</i>
                </button>
            `);
        }
        
        // 实时语音生成按钮（备用或重新生成）
        controlsHtml.push(`
            <button onclick="playTextAsVoice('${text.replace(/'/g, "\\'")}')">
                <i class="material-icons">volume_up</i>
            </button>
            <button onclick="copyText('${text.replace(/'/g, "\\'")}')">
                <i class="material-icons">content_copy</i>
            </button>
        `);
    } else {
        // 用户消息也可以复制
        controlsHtml.push(`
            <button onclick="copyText('${text.replace(/'/g, "\\'")}')">
                <i class="material-icons">content_copy</i>
            </button>
        `);
    }
    
    messageContent += `
        <div class="message-controls">
            ${controlsHtml.join('')}
        </div>
    `;
    
    messageDiv.innerHTML = messageContent;
    chatMessages.appendChild(messageDiv);
    
    // 滚动到底部
    chatMessages.scrollTop = chatMessages.scrollHeight;
}

// 播放音频 - 支持不同格式
function playAudio(audioBase64, format = 'mp3') {
    try {
        // 检查音频数据
        if (!audioBase64 || audioBase64.trim() === '') {
            console.warn('音频数据为空，无法播放');
            showToast('❌ 音频数据为空，无法播放', 'error');
            return;
        }
        
        const audioPlayer = document.getElementById('audio-player');
        if (!audioPlayer) {
            console.error('找不到音频播放器元素');
            showToast('❌ 音频播放器未找到', 'error');
            return;
        }
        
        const mimeType = format === 'wav' ? 'audio/wav' : 'audio/mp3';
        
        // 清理之前的事件监听器
        audioPlayer.onloadstart = null;
        audioPlayer.oncanplay = null;
        audioPlayer.onerror = null;
        audioPlayer.onended = null;
        
        // 设置新的事件监听器
        audioPlayer.onloadstart = () => {
            console.log('🎵 音频开始加载...');
            showToast('🎵 音频加载中...', 'info');
        };
        
        audioPlayer.oncanplay = () => {
            console.log('🔊 音频可以播放了');
            showToast('🔊 开始播放ChatGLM语音', 'success');
        };
        
        audioPlayer.onerror = (e) => {
            console.error('❌ 音频播放错误:', e);
            showToast('❌ 音频播放失败', 'error');
        };
        
        audioPlayer.onended = () => {
            console.log('✅ 音频播放完成');
            showToast('✅ 语音播放完成', 'success');
        };
        
        // 设置音频源
        audioPlayer.src = `data:${mimeType};base64,${audioBase64}`;
        audioPlayer.load(); // 重新加载音频
        
        // 播放音频
        const playPromise = audioPlayer.play();
        if (playPromise !== undefined) {
            playPromise.then(() => {
                console.log('✅ 音频播放开始');
            }).catch(error => {
                console.error('❌ 音频播放失败:', error);
                showToast('❌ 音频播放失败: ' + error.message, 'error');
            });
        }
        
    } catch (error) {
        console.error('❌ 音频处理失败:', error);
        showToast('❌ 音频处理失败: ' + error.message, 'error');
    }
}

// 重播消息中的音频
function replayMessageAudio(messageId) {
    try {
        console.log('🔄 尝试重播音频，消息ID:', messageId);
        
        const messageElement = document.querySelector(`[data-message-id="${messageId}"]`);
        if (!messageElement) {
            console.error('❌ 找不到消息元素，ID:', messageId);
            showToast('❌ 消息不存在', 'error');
            return;
        }
        
        const audioData = messageElement.getAttribute('data-audio');
        if (!audioData || audioData.trim() === '') {
            console.warn('⚠️ 该消息没有音频数据');
            showToast('⚠️ 该消息没有音频数据，无法重播', 'warning');
            return;
        }
        
        console.log('🎵 找到音频数据，开始播放...');
        
        // 播放保存的ChatGLM音频
        playAudio(audioData, 'wav');
        showToast('🔄 重播ChatGLM语音', 'info');
        
    } catch (error) {
        console.error('❌ 重播音频时出错:', error);
        showToast('❌ 重播音频失败: ' + error.message, 'error');
    }
}

// 朗读文本 - 优先使用ChatGLM，回退到浏览器TTS
async function playTextAsVoice(text) {
    try {
        // 首先尝试使用ChatGLM语音生成
        showToast('正在生成语音...', 'info');
        
        const response = await fetch(`${API_BASE_URL}/api/chatglm-tts`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ text: text })
        });
        
        if (response.ok) {
            const result = await response.json();
            if (result.success && result.audio_data) {
                // 播放ChatGLM生成的语音
                const audioPlayer = document.getElementById('audio-player');
                audioPlayer.src = `data:audio/wav;base64,${result.audio_data}`;
                
                audioPlayer.onloadstart = () => showToast('语音加载中...', 'info');
                audioPlayer.oncanplay = () => showToast('开始播放语音', 'success');
                audioPlayer.onerror = () => {
                    showToast('语音播放失败，尝试备用方案', 'warning');
                    fallbackToNativeTTS(text);
                };
                
                await audioPlayer.play();
                return;
            }
        }
        
        // ChatGLM失败，回退到浏览器TTS
        console.log('ChatGLM语音生成失败，使用浏览器TTS');
        fallbackToNativeTTS(text);
        
    } catch (error) {
        console.error('ChatGLM语音生成错误:', error);
        showToast('语音生成失败，使用浏览器TTS', 'warning');
        fallbackToNativeTTS(text);
    }
}

// 备用的浏览器原生TTS
function fallbackToNativeTTS(text) {
    if ('speechSynthesis' in window) {
        const utterance = new SpeechSynthesisUtterance(text);
        utterance.lang = 'zh-CN';
        utterance.rate = 0.9;  // 稍微慢一点
        utterance.pitch = 1.0;
        utterance.volume = 0.8;
        
        utterance.onstart = () => showToast('开始语音播放', 'success');
        utterance.onerror = () => showToast('语音播放失败', 'error');
        
        speechSynthesis.speak(utterance);
    } else {
        showToast('您的浏览器不支持语音功能', 'error');
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
                    <i class="material-icons">flight_takeoff</i>
                    <p>欢迎使用微分智飞无人机语音操控系统！</p>
                    <p>您可以通过语音、文字或图片指令控制无人机。</p>
                </div>
            `;
            showToast('操控记录已清除', 'success');
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

// 加载可用模型列表
async function loadAvailableModels() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/config/model`);
        const result = await response.json();
        
        if (result.success && result.available_models) {
            const modelSelect = document.getElementById('model-select');
            const currentSelection = modelSelect.value;
            
            // 清空现有选项
            modelSelect.innerHTML = '';
            
            // 按分类添加模型选项
            const models = result.available_models;
            
            // 分类模型
            const geminiModels = models.filter(m => m.startsWith('gemini-'));
            const gptModels = models.filter(m => m.startsWith('gpt-') && !m.includes('realtime'));
            const doubaoModels = models.filter(m => m.startsWith('doubao-'));
            const realtimeModels = models.filter(m => m.includes('realtime') || m.includes('glm-realtime'));
            
            // 添加Gemini模型
            if (geminiModels.length > 0) {
                const geminiGroup = document.createElement('optgroup');
                geminiGroup.label = 'Gemini Models';
                geminiModels.forEach(model => {
                    const option = document.createElement('option');
                    option.value = model;
                    option.textContent = model;
                    geminiGroup.appendChild(option);
                });
                modelSelect.appendChild(geminiGroup);
            }
            
            // 添加GPT模型
            if (gptModels.length > 0) {
                const gptGroup = document.createElement('optgroup');
                gptGroup.label = 'OpenAI GPT Models';
                gptModels.forEach(model => {
                    const option = document.createElement('option');
                    option.value = model;
                    option.textContent = model.includes('gpt-4o-mini') ? 'GPT-4o-mini (OpenAI)' : 
                                       model.includes('gpt-4o') ? 'GPT-4o (OpenAI)' : model;
                    gptGroup.appendChild(option);
                });
                modelSelect.appendChild(gptGroup);
            }
            
            // 添加豆包模型
            if (doubaoModels.length > 0) {
                const doubaoGroup = document.createElement('optgroup');
                doubaoGroup.label = '豆包 Models (字节跳动)';
                doubaoModels.forEach(model => {
                    const option = document.createElement('option');
                    option.value = model;
                    // 为豆包模型提供友好的显示名称
                    let displayName = model;
                    if (model === 'doubao-seed-1-6-vision-250815') {
                        displayName = '豆包-1.6-视觉版 (字节跳动)';
                    } else if (model === 'doubao-lite-4k') {
                        displayName = '豆包-Lite-4k (字节跳动)';
                    } else if (model === 'doubao-pro-4k') {
                        displayName = '豆包-Pro-4k (字节跳动)';
                    } else if (model === 'doubao-pro-32k') {
                        displayName = '豆包-Pro-32k (字节跳动)';
                    } else if (model === 'doubao-pro-128k') {
                        displayName = '豆包-Pro-128k (字节跳动)';
                    }
                    option.textContent = displayName;
                    doubaoGroup.appendChild(option);
                });
                modelSelect.appendChild(doubaoGroup);
            }
            
            // 添加实时模型
            if (realtimeModels.length > 0) {
                const realtimeGroup = document.createElement('optgroup');
                realtimeGroup.label = 'Realtime Models (实时语音)';
                realtimeModels.forEach(model => {
                    const option = document.createElement('option');
                    option.value = model;
                    if (model === 'gpt-4o-realtime-preview') {
                        option.textContent = 'GPT-4o-Realtime (OpenAI)';
                    } else if (model === 'glm-realtime') {
                        option.textContent = 'GLM-Realtime (智谱)';
                    } else {
                        option.textContent = model;
                    }
                    // 为实时模型添加特殊样式标识
                    option.className = 'realtime-model';
                    realtimeGroup.appendChild(option);
                });
                modelSelect.appendChild(realtimeGroup);
            }
            
            // 恢复之前的选择或设置默认值
            if (currentSelection && models.includes(currentSelection)) {
                modelSelect.value = currentSelection;
            } else if (models.includes(currentModel)) {
                modelSelect.value = currentModel;
            } else if (models.length > 0) {
                modelSelect.value = models[0];
                currentModel = models[0];
            }
            
            console.log('模型列表加载成功:', models);
        } else {
            console.error('加载模型列表失败:', result.error || '未知错误');
            showToast('⚠️ 无法加载模型列表，使用默认配置', 'warning');
        }
    } catch (error) {
        console.error('加载模型列表异常:', error);
        showToast('❌ 模型列表加载失败', 'error');
    }
}

// 检查是否为实时模型
function isRealtimeModel(modelName) {
    const realtimeModels = ['gpt-4o-realtime-preview', 'glm-realtime'];
    return realtimeModels.includes(modelName);
}

// 切换模型
async function changeModel() {
    const modelSelect = document.getElementById('model-select');
    const selectedModel = modelSelect.value;
    
    // 检查是否为实时模型
    if (isRealtimeModel(selectedModel)) {
        // 为实时模型显示特殊提示
        showToast(`🎤 已选择实时语音模型: ${selectedModel}`, 'info');
        showToast(`🔄 正在自动连接实时模型...`, 'info');
        currentModel = selectedModel;
        console.log(`实时模型选择: ${selectedModel}`);
        
        // 显示实时模型控制面板
        showRealtimeControls(true);
        
        // 自动连接实时模型
        await autoConnectRealtimeModel();
        return;
    } else {
        // 隐藏实时模型控制面板
        showRealtimeControls(false);
        
        // 切换回普通对话界面
        switchToNormalInterface();
    }
    
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
            showToast(`✅ 模型已切换到: ${selectedModel}`, 'success');
            console.log(`模型切换成功: ${selectedModel}`);
            
            // 控制豆包深度思考开关的显示
            updateThinkingControlVisibility(selectedModel);
        } else {
            console.error('模型切换失败:', result.error);
            
            // 提供更友好的错误提示
            let errorMessage = result.error || '未知错误';
            if (errorMessage.includes('网络连接')) {
                errorMessage = '🌐 网络连接问题，请检查网络设置后重试';
            } else if (errorMessage.includes('不可用')) {
                errorMessage = '⚠️ 所选模型暂时不可用，请稍后重试';
            } else {
                errorMessage = `❌ 模型切换失败: ${errorMessage}`;
            }
            
            showToast(errorMessage, 'error');
            // 恢复到之前的选择
            modelSelect.value = currentModel;
        }
    } catch (error) {
        console.error('模型切换异常:', error);
        
        let errorMessage = '❌ 模型切换失败';
        if (error.message.includes('fetch')) {
            errorMessage = '🌐 网络连接问题，请检查服务器状态';
        } else if (error.message.includes('timeout')) {
            errorMessage = '⏱️ 请求超时，请稍后重试';
        } else {
            errorMessage = `❌ 模型切换异常: ${error.message}`;
        }
        
        showToast(errorMessage, 'error');
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
        
        // 处理图片信息
        let imageDisplay = '';
        if (msg.images_info && msg.images_info.length > 0) {
            imageDisplay = '<div class="message-images">';
            msg.images_info.forEach((img, index) => {
                if (img.data) {
                    // 确保data URL格式正确
                    const dataUrl = img.data.startsWith('data:') ? img.data : `data:image/jpeg;base64,${img.data}`;
                    imageDisplay += `
                        <img src="${dataUrl}" 
                             alt="图片${index + 1}" 
                             title="${img.filename || `图片${index + 1}`} (${img.format || 'unknown'} ${img.size || ''})"
                             style="max-width: 100px; max-height: 80px; margin: 2px; border-radius: 4px; cursor: pointer;"
                             onclick="viewImage('${dataUrl}', '${img.filename || `图片${index + 1}`}'); event.stopPropagation();" />
                    `;
                }
            });
            imageDisplay += `<div class="image-count-badge">${msg.images_info.length} 张图片</div></div>`;
        }
        
        html += `
            <div class="incoming-message ${isUnread ? 'unread' : ''} ${priorityClass}" 
                 onclick="selectIncomingMessage(${msg.id})">
                <div class="message-header">
                    <span class="message-sender">${msg.sender || 'unknown'}</span>
                    <span class="message-time">${timeStr}</span>
                    ${msg.image_count ? `<span class="image-badge">${msg.image_count}📷</span>` : ''}
                </div>
                <div class="message-content">${msg.text}</div>
                ${imageDisplay}
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
        
        // 如果有图片，也将图片添加到当前的图片队列
        if (message.images_info && message.images_info.length > 0) {
            // 清空当前选择的图片
            selectedImages = [];
            currentImages = []; // 也清空currentImages
            
            // 添加传入的图片到选择队列和当前图片队列
            message.images_info.forEach((img, index) => {
                if (img.data) {
                    const dataUrl = img.data.startsWith('data:') ? img.data : `data:image/jpeg;base64,${img.data}`;
                    const imageData = {
                        file: null, // 外部传入的图片没有file对象
                        data: dataUrl,
                        name: img.filename || `传入图片${index + 1}`,
                        size: img.size || 'unknown'
                    };
                    
                    // 添加到selectedImages（用于预览）
                    selectedImages.push(imageData);
                    
                    // 添加到currentImages（用于发送）
                    currentImages.push({
                        data: dataUrl,
                        type: img.format || 'image/jpeg'
                    });
                }
            });
            
            // 更新图片预览
            updateImagePreview();
            
            showToast(`消息已转入对话框 (包含${message.images_info.length}张图片)`, 'success');
        } else {
            showToast('消息已转入对话框', 'success');
        }
        
        // 标记为已读
        if (!message.read) {
            markMessageRead(messageId);
        }
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

// ===================记忆块管理功能===================

// 记忆存储键名
const MEMORY_STORAGE_KEY = 'gemini_chat_memories';

// 加载记忆列表
function loadMemories() {
    try {
        const stored = localStorage.getItem(MEMORY_STORAGE_KEY);
        return stored ? JSON.parse(stored) : [];
    } catch (error) {
        console.error('加载记忆失败:', error);
        return [];
    }
}

// 保存记忆列表
function saveMemories(memories) {
    try {
        localStorage.setItem(MEMORY_STORAGE_KEY, JSON.stringify(memories));
    } catch (error) {
        console.error('保存记忆失败:', error);
        showToast('保存记忆失败', 'error');
    }
}

// 渲染记忆列表
function renderMemories(filteredMemories = null) {
    const memories = filteredMemories || loadMemories();
    const memoryList = document.getElementById('memory-list');
    
    if (memories.length === 0) {
        memoryList.innerHTML = `
            <div class="no-memories">
                <i class="material-icons">memory</i>
                <p>暂无保存的指令记忆</p>
                <small>在操控对话中选择内容保存为指令记忆</small>
            </div>
        `;
        return;
    }
    
    // 按时间倒序排列
    memories.sort((a, b) => new Date(b.created_at) - new Date(a.created_at));
    
    memoryList.innerHTML = memories.map(memory => `
        <div class="memory-item" data-memory-id="${memory.id}" onclick="loadMemory('${memory.id}')">
            <h4>
                ${memory.title}
                <div class="memory-actions">
                    <button class="edit-btn" onclick="event.stopPropagation(); editMemory('${memory.id}')" title="编辑">
                        <i class="material-icons">edit</i>
                    </button>
                    <button class="delete-btn" onclick="event.stopPropagation(); deleteMemory('${memory.id}')" title="删除">
                        <i class="material-icons">delete</i>
                    </button>
                </div>
            </h4>
            <div class="memory-content">${memory.content}</div>
            <div class="memory-meta">
                <span>${memory.sender === 'user' ? '操控员' : '系统AI'}</span>
                <span>${new Date(memory.created_at).toLocaleDateString()}</span>
            </div>
        </div>
    `).join('');
}

// 保存消息为记忆
function saveAsMemory(messageId, sender) {
    const messageElement = document.querySelector(`[data-message-id="${messageId}"]`);
    if (!messageElement) {
        showToast('消息不存在', 'error');
        return;
    }
    
    // 提取消息内容
    const messageContentElement = messageElement.querySelector('.message-content');
    const imageElements = messageElement.querySelectorAll('.image-content img');
    
    let content = '';
    let title = '';
    
    // 处理文本内容
    if (messageContentElement) {
        content = messageContentElement.textContent || messageContentElement.innerText;
        title = content.length > 30 ? content.substring(0, 30) + '...' : content;
    }
    
    // 处理图片内容
    if (imageElements.length > 0) {
        const imageInfo = `[包含${imageElements.length}张图片]`;
        content = content ? `${content}\n${imageInfo}` : imageInfo;
        if (!title) {
            title = imageInfo;
        }
    }
    
    if (!content) {
        showToast('没有可保存的内容', 'warning');
        return;
    }
    
    // 创建记忆对象
    const memory = {
        id: Date.now().toString(),
        title: title || '未命名记忆',
        content: content,
        sender: sender,
        created_at: new Date().toISOString(),
        updated_at: new Date().toISOString(),
        message_id: messageId
    };
    
    // 保存到存储
    const memories = loadMemories();
    memories.push(memory);
    saveMemories(memories);
    
    // 更新显示
    renderMemories();
    showToast('记忆已保存', 'success');
    
    // 高亮保存的消息
    messageElement.style.background = 'rgba(76, 175, 80, 0.1)';
    setTimeout(() => {
        messageElement.style.background = '';
    }, 2000);
}

// 搜索记忆
function searchMemories() {
    const searchTerm = document.getElementById('memory-search').value.toLowerCase();
    if (!searchTerm) {
        renderMemories();
        return;
    }
    
    const memories = loadMemories();
    const filtered = memories.filter(memory => 
        memory.title.toLowerCase().includes(searchTerm) ||
        memory.content.toLowerCase().includes(searchTerm)
    );
    
    renderMemories(filtered);
}

// 加载记忆内容到输入框
function loadMemory(memoryId) {
    const memories = loadMemories();
    const memory = memories.find(m => m.id === memoryId);
    
    if (!memory) {
        showToast('记忆不存在', 'error');
        return;
    }
    
    // 将指令记忆内容填入输入框
    const textInput = document.getElementById('text-input');
    textInput.value = memory.content;
    textInput.focus();
    
    // 高亮选中的记忆
    document.querySelectorAll('.memory-item').forEach(item => {
        item.classList.remove('selected');
    });
    document.querySelector(`[data-memory-id="${memoryId}"]`).classList.add('selected');
    
    showToast('指令记忆已加载到输入框', 'success');
}

// 编辑记忆
function editMemory(memoryId) {
    const memories = loadMemories();
    const memory = memories.find(m => m.id === memoryId);
    
    if (!memory) {
        showToast('记忆不存在', 'error');
        return;
    }
    
    const newTitle = prompt('编辑标题:', memory.title);
    if (newTitle === null) return; // 用户取消
    
    const newContent = prompt('编辑内容:', memory.content);
    if (newContent === null) return; // 用户取消
    
    // 更新记忆
    memory.title = newTitle || '未命名记忆';
    memory.content = newContent || '';
    memory.updated_at = new Date().toISOString();
    
    // 保存更改
    saveMemories(memories);
    renderMemories();
    showToast('记忆已更新', 'success');
}

// 删除记忆
function deleteMemory(memoryId) {
    if (!confirm('确定要删除这条记忆吗？')) {
        return;
    }
    
    const memories = loadMemories();
    const filteredMemories = memories.filter(m => m.id !== memoryId);
    
    saveMemories(filteredMemories);
    renderMemories();
    showToast('记忆已删除', 'success');
}

// 新建记忆
function addNewMemory() {
    const title = prompt('请输入记忆标题:');
    if (!title) return;
    
    const content = prompt('请输入记忆内容:');
    if (!content) return;
    
    const memory = {
        id: Date.now().toString(),
        title: title,
        content: content,
        sender: 'user',
        created_at: new Date().toISOString(),
        updated_at: new Date().toISOString(),
        message_id: null
    };
    
    const memories = loadMemories();
    memories.push(memory);
    saveMemories(memories);
    renderMemories();
    showToast('新记忆已创建', 'success');
}

// 导出记忆
function exportMemories() {
    const memories = loadMemories();
    if (memories.length === 0) {
        showToast('没有记忆可导出', 'warning');
        return;
    }
    
    const dataStr = JSON.stringify(memories, null, 2);
    const dataBlob = new Blob([dataStr], {type: 'application/json'});
    
    const link = document.createElement('a');
    link.href = URL.createObjectURL(dataBlob);
    link.download = `gemini_memories_${new Date().toISOString().split('T')[0]}.json`;
    link.click();
    
    showToast('记忆已导出', 'success');
}

// 导入记忆
function importMemories() {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json';
    
    input.onchange = function(event) {
        const file = event.target.files[0];
        if (!file) return;
        
        const reader = new FileReader();
        reader.onload = function(e) {
            try {
                const importedMemories = JSON.parse(e.target.result);
                if (!Array.isArray(importedMemories)) {
                    throw new Error('无效的记忆文件格式');
                }
                
                const existingMemories = loadMemories();
                const allMemories = [...existingMemories, ...importedMemories];
                
                saveMemories(allMemories);
                renderMemories();
                showToast(`成功导入 ${importedMemories.length} 条记忆`, 'success');
                
            } catch (error) {
                console.error('导入记忆失败:', error);
                showToast('导入失败: ' + error.message, 'error');
            }
        };
        reader.readAsText(file);
    };
    
    input.click();
}

// ChatGLM语速控制函数
let currentSpeechRate = 0.8; // 默认语速

// 更新语速显示和发送到服务器
async function updateSpeechRate() {
    const slider = document.getElementById('speech-rate');
    const value = parseFloat(slider.value);
    currentSpeechRate = value;
    
    // 更新显示
    document.getElementById('speech-rate-value').textContent = value + 'x';
    
    // 更新预设按钮状态
    updatePresetButtons(value);
    
    // 发送到服务器
    await sendSpeechRateToServer(value);
}

// 设置语速预设值
async function setSpeechRate(rate) {
    const slider = document.getElementById('speech-rate');
    slider.value = rate;
    await updateSpeechRate();
}

// 更新预设按钮状态
function updatePresetButtons(currentRate) {
    const buttons = document.querySelectorAll('.preset-btn');
    buttons.forEach(btn => {
        btn.classList.remove('active');
        const btnRate = parseFloat(btn.onclick.toString().match(/setSpeechRate\(([^)]+)\)/)[1]);
        if (Math.abs(btnRate - currentRate) < 0.05) {
            btn.classList.add('active');
        }
    });
}

// 发送语速设置到服务器
async function sendSpeechRateToServer(rate) {
    try {
        const response = await fetch(`${API_BASE_URL}/api/chatglm-speech-rate`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ speech_rate: rate })
        });
        
        const result = await response.json();
        
        if (result.success) {
            document.getElementById('speech-rate-status').textContent = `✅ 语速已设置为: ${rate}x`;
            document.getElementById('speech-rate-status').className = 'config-status success';
            console.log('语速设置成功:', rate);
        } else {
            throw new Error(result.error || '设置失败');
        }
    } catch (error) {
        console.error('设置语速失败:', error);
        document.getElementById('speech-rate-status').textContent = `❌ 设置失败: ${error.message}`;
        document.getElementById('speech-rate-status').className = 'config-status error';
        showToast('语速设置失败: ' + error.message, 'error');
    }
}

// 豆包深度思考控制函数
let currentThinkingEnabled = true; // 默认启用深度思考

// 更新豆包深度思考设置
async function updateDoubaoThinking() {
    const checkbox = document.getElementById('doubao-thinking');
    const enabled = checkbox.checked;
    currentThinkingEnabled = enabled;
    
    // 更新状态显示
    const statusElement = document.getElementById('thinking-status');
    statusElement.textContent = enabled ? '已启用' : '已禁用';
    statusElement.style.color = enabled ? '#28a745' : '#dc3545';
    
    try {
        const response = await fetch(`${API_BASE_URL}/api/doubao-thinking`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                thinking_enabled: enabled
            })
        });
        
        const result = await response.json();
        if (result.success) {
            const statusMsg = enabled ? '深度思考已启用' : '深度思考已禁用';
            showToast(statusMsg, 'success');
            updateThinkingConfigStatus(`✅ ${result.message}`, 'success');
        } else {
            showToast('深度思考设置失败: ' + result.error, 'error');
            updateThinkingConfigStatus(`❌ ${result.error}`, 'error');
        }
    } catch (error) {
        console.error('设置深度思考失败:', error);
        showToast('深度思考设置失败: ' + error.message, 'error');
        updateThinkingConfigStatus(`❌ 网络错误: ${error.message}`, 'error');
    }
}

// 从服务器加载当前深度思考设置
async function loadDoubaoThinkingConfig() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/doubao-thinking`);
        const result = await response.json();
        
        if (result.success) {
            currentThinkingEnabled = result.thinking_enabled;
            
            // 更新UI
            const checkbox = document.getElementById('doubao-thinking');
            checkbox.checked = currentThinkingEnabled;
            
            const statusElement = document.getElementById('thinking-status');
            statusElement.textContent = currentThinkingEnabled ? '已启用' : '已禁用';
            statusElement.style.color = currentThinkingEnabled ? '#28a745' : '#dc3545';
            
            updateThinkingConfigStatus(`✅ 深度思考配置已加载 (${currentThinkingEnabled ? '启用' : '禁用'})`, 'success');
            console.log('豆包深度思考配置加载成功:', result);
        } else {
            console.error('加载深度思考配置失败:', result.error);
            updateThinkingConfigStatus(`❌ 加载失败: ${result.error}`, 'error');
        }
    } catch (error) {
        console.error('加载深度思考配置时出错:', error);
        updateThinkingConfigStatus(`❌ 网络错误: ${error.message}`, 'error');
        showToast('加载深度思考配置失败', 'error');
    }
}

// 更新深度思考配置状态显示
function updateThinkingConfigStatus(message, type) {
    const statusElement = document.getElementById('thinking-config-status');
    statusElement.textContent = message;
    statusElement.className = `config-status ${type}`;
    
    // 3秒后清除状态
    setTimeout(() => {
        statusElement.textContent = '';
        statusElement.className = 'config-status';
    }, 3000);
}

// 同步当前模型状态
async function syncCurrentModel() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/config/model`);
        const result = await response.json();
        
        if (result.success && result.model) {
            const serverModel = result.model;
            console.log(`从服务器获取当前模型: ${serverModel}`);
            
            // 更新前端状态
            currentModel = serverModel;
            
            // 更新模型选择器
            const modelSelect = document.getElementById('model-select');
            if (modelSelect) {
                modelSelect.value = serverModel;
            }
            
            // 更新深度思考控制可见性
            updateThinkingControlVisibility(serverModel);
            
            console.log(`前端模型状态已同步: ${serverModel}`);
        } else {
            console.error('获取当前模型失败:', result.error);
        }
    } catch (error) {
        console.error('同步当前模型时出错:', error);
    }
}

// 控制豆包深度思考开关的显示
function updateThinkingControlVisibility(modelName) {
    const thinkingRow = document.getElementById('doubao-thinking-row');
    
    if (!thinkingRow) {
        console.error('找不到豆包深度思考控制行');
        return;
    }
    
    // 检查是否为豆包模型
    const isDoubaoModel = modelName && modelName.toLowerCase().includes('doubao');
    
    // 显示或隐藏豆包深度思考控制
    if (isDoubaoModel) {
        thinkingRow.style.display = 'block';
        console.log('显示豆包深度思考控制');
    } else {
        thinkingRow.style.display = 'none';
        console.log('隐藏豆包深度思考控制');
    }
}

// 从服务器加载当前语速设置
async function loadSpeechRate() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/chatglm-speech-rate`);
        const result = await response.json();
        
        if (result.success) {
            const rate = result.speech_rate;
            document.getElementById('speech-rate').value = rate;
            document.getElementById('speech-rate-value').textContent = rate + 'x';
            currentSpeechRate = rate;
            updatePresetButtons(rate);
            document.getElementById('speech-rate-status').textContent = `✅ 当前语速: ${rate}x`;
            document.getElementById('speech-rate-status').className = 'config-status success';
        }
    } catch (error) {
        console.error('加载语速设置失败:', error);
        document.getElementById('speech-rate-status').textContent = '❌ 加载语速设置失败';
        document.getElementById('speech-rate-status').className = 'config-status error';
    }
}

// 查看图片函数
function viewImage(imageUrl, imageName) {
    // 创建模态框查看图片
    const modal = document.createElement('div');
    modal.style.cssText = `
        position: fixed; top: 0; left: 0; width: 100%; height: 100%;
        background: rgba(0,0,0,0.8); z-index: 10000; display: flex;
        align-items: center; justify-content: center; cursor: pointer;
    `;
    
    const img = document.createElement('img');
    img.src = imageUrl;
    img.style.cssText = `
        max-width: 90%; max-height: 90%; border-radius: 8px;
        box-shadow: 0 4px 20px rgba(0,0,0,0.5);
    `;
    
    const caption = document.createElement('div');
    caption.textContent = imageName;
    caption.style.cssText = `
        position: absolute; bottom: 20px; left: 50%; transform: translateX(-50%);
        color: white; background: rgba(0,0,0,0.7); padding: 10px 20px;
        border-radius: 20px; font-size: 14px;
    `;
    
    modal.appendChild(img);
    modal.appendChild(caption);
    document.body.appendChild(modal);
}

// ==================== 实时模型控制功能 ====================

// 显示/隐藏实时模型控制面板
function showRealtimeControls(show) {
    const realtimeControls = document.getElementById('realtime-controls');
    if (realtimeControls) {
        realtimeControls.style.display = show ? 'block' : 'none';
    }
}

// 自动连接实时模型（模型切换时调用）
async function autoConnectRealtimeModel() {
    if (!isRealtimeModel(currentModel)) {
        return false;
    }
    
    try {
        // 更新输入框提示
        const textInput = document.getElementById('text-input');
        if (textInput) {
            textInput.placeholder = '🔄 正在连接实时模型，请稍候...';
        }
        
        // 执行连接
        const success = await connectRealtimeModel();
        
        if (success) {
            // 连接成功，切换到实时对话界面
            switchToRealtimeInterface();
            showToast(`✅ ${currentModel} 自动连接成功，现在可以直接对话！`, 'success');
        } else {
            // 连接失败，显示手动连接选项
            if (textInput) {
                textInput.placeholder = '❌ 自动连接失败，请点击上方"连接实时模型"按钮手动连接...';
            }
            showToast(`❌ ${currentModel} 自动连接失败，请手动连接`, 'error');
        }
        
        return success;
    } catch (error) {
        console.error('自动连接实时模型失败:', error);
        showToast(`❌ 自动连接失败: ${error.message}`, 'error');
        return false;
    }
}

// 切换到实时对话界面
function switchToRealtimeInterface() {
    const textInput = document.getElementById('text-input');
    if (textInput) {
        textInput.placeholder = '🎤 实时模型已连接！直接输入消息开始对话...';
        textInput.style.borderColor = '#ff6b35'; // 橙色边框表示实时模式
        textInput.style.boxShadow = '0 0 5px rgba(255, 107, 53, 0.3)';
    }
    
    // 更新发送按钮样式
    const sendBtn = document.getElementById('send-btn');
    if (sendBtn) {
        sendBtn.style.backgroundColor = '#ff6b35';
        sendBtn.title = '发送到实时模型';
    }
    
    // 在聊天区域添加提示消息
    addSystemMessage(`🎤 已切换到 ${currentModel} 实时模式`);
}

// 切换回普通对话界面
function switchToNormalInterface() {
    const textInput = document.getElementById('text-input');
    if (textInput) {
        textInput.placeholder = '输入无人机控制指令...';
        textInput.style.borderColor = ''; // 恢复默认边框
        textInput.style.boxShadow = '';
    }
    
    // 恢复发送按钮样式
    const sendBtn = document.getElementById('send-btn');
    if (sendBtn) {
        sendBtn.style.backgroundColor = '';
        sendBtn.title = '';
    }
}

// 添加系统消息到聊天区域
function addSystemMessage(message) {
    const chatMessages = document.getElementById('chat-messages');
    if (chatMessages) {
        const systemMsg = document.createElement('div');
        systemMsg.className = 'system-message';
        systemMsg.innerHTML = `
            <div class="system-content">
                <i class="material-icons">info</i>
                <span>${message}</span>
            </div>
        `;
        chatMessages.appendChild(systemMsg);
        chatMessages.scrollTop = chatMessages.scrollHeight;
    }
}

// 🎤 一键开始语音对话
async function startVoiceChat() {
    const currentModel = getCurrentModel();
    if (!isRealtimeModel(currentModel)) {
        showToast('请先选择实时模型（GPT-4o-Realtime 或 GLM-Realtime）', 'warning');
        return;
    }

    try {
        // 更新状态显示
        updateVoiceChatStatus('🟡 正在连接...', 'connecting');
        
        // 1. 先连接后端
        console.log(`开始语音对话: ${currentModel}`);
        const response = await fetch(`${API_BASE_URL}/api/realtime/connect`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ model: currentModel })
        });
        
        const result = await response.json();
        
        if (result.success) {
            // 2. 建立WebRTC/WebSocket连接
            const connected = await createRealtimeWebRTC(currentModel);
            
            if (connected) {
                // 3. 连接成功，开始语音对话
                updateVoiceChatStatus('🟢 语音对话已开始 - 请直接说话', 'active');
                showToast(`🎤 ${currentModel} 语音对话已开始！直接说话即可`, 'success');
                
                // 更新按钮状态
                document.getElementById('voice-chat-btn').disabled = true;
                document.getElementById('stop-voice-chat-btn').disabled = false;
                
                // 切换到实时界面
                switchToRealtimeInterface();
                
                // 添加使用指南
                addSystemMessage('🎤 语音对话已开始！直接对着麦克风说话，AI会实时回复');
                addSystemMessage('💡 说话时会显示"🎤 检测到语音输入..."，AI回复会实时显示');
                
                return true;
            } else {
                throw new Error('WebRTC连接失败');
            }
        } else {
            throw new Error(result.error || '后端连接失败');
        }
        
    } catch (error) {
        console.error('开始语音对话失败:', error);
        updateVoiceChatStatus('🔴 连接失败 - 点击重试', 'error');
        showToast(`❌ 语音对话启动失败: ${error.message}`, 'error');
        
        // 恢复按钮状态
        document.getElementById('voice-chat-btn').disabled = false;
        document.getElementById('stop-voice-chat-btn').disabled = true;
        
        return false;
    }
}

// ⏹️ 停止语音对话
async function stopVoiceChat() {
    const currentModel = getCurrentModel();
    
    try {
        updateVoiceChatStatus('🟡 正在断开连接...', 'disconnecting');
        
        // 1. 断开WebRTC连接
        disconnectRealtimeWebRTC();
        
        // 2. 断开后端连接
        const response = await fetch(`${API_BASE_URL}/api/realtime/disconnect`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ model: currentModel })
        });
        
        const result = await response.json();
        
        if (result.success) {
            updateVoiceChatStatus('🔴 语音对话已停止', 'stopped');
            showToast('⏹️ 语音对话已停止', 'info');
            
            // 恢复按钮状态
            document.getElementById('voice-chat-btn').disabled = false;
            document.getElementById('stop-voice-chat-btn').disabled = true;
            
            // 切换回普通界面
            switchToNormalInterface();
            
            addSystemMessage('⏹️ 语音对话已结束');
            
            return true;
        } else {
            throw new Error(result.error || '断开连接失败');
        }
        
    } catch (error) {
        console.error('停止语音对话失败:', error);
        showToast(`⚠️ 断开连接时出现问题: ${error.message}`, 'warning');
        
        // 强制恢复状态
        updateVoiceChatStatus('🔴 已强制停止', 'stopped');
        document.getElementById('voice-chat-btn').disabled = false;
        document.getElementById('stop-voice-chat-btn').disabled = true;
        disconnectRealtimeWebRTC();
        switchToNormalInterface();
        
        return false;
    }
}

// 更新语音对话状态显示
function updateVoiceChatStatus(message, status) {
    const statusElement = document.getElementById('voice-chat-status');
    if (statusElement) {
        statusElement.innerHTML = `<span>${message}</span>`;
        
        // 根据状态设置颜色
        switch (status) {
            case 'connecting':
                statusElement.style.color = '#ffc107';
                break;
            case 'active':
                statusElement.style.color = '#28a745';
                break;
            case 'error':
                statusElement.style.color = '#dc3545';
                break;
            case 'stopped':
                statusElement.style.color = '#6c757d';
                break;
            default:
                statusElement.style.color = '';
        }
    }
}

// 连接实时模型 (兼容旧版)
async function connectRealtimeModel() {
    if (!isRealtimeModel(currentModel)) {
        showToast('请先选择实时语音模型', 'warning');
        return;
    }
    
    try {
        updateRealtimeStatus('正在连接实时模型...', 'info');
        
        const response = await fetch(`${API_BASE_URL}/api/realtime/connect`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ model: currentModel })
        });
        
        const result = await response.json();
        
        if (result.success) {
            showToast(`✅ ${currentModel} 连接成功`, 'success');
            showToast(`🎉 现在可以在下方输入框正常对话了！`, 'info');
            updateRealtimeStatus(`连接成功: ${result.message}`, 'success');
            
            // 切换到实时对话界面
            switchToRealtimeInterface();
            
            // 启动实时响应轮询
            startRealtimePolling(currentModel);
            
            // 尝试创建WebRTC/WebSocket连接以获得真实的实时响应
            setTimeout(async () => {
                const connected = await createRealtimeWebRTC(currentModel);
                if (!connected) {
                    console.error('实时连接失败，回退到轮询模式');
                    showToast('实时连接失败，使用HTTP轮询模式', 'warning');
                }
            }, 1000); // 延迟1秒确保后端准备就绪
            
            // 更新按钮状态
            document.getElementById('realtime-connect-btn').disabled = true;
            document.getElementById('realtime-disconnect-btn').disabled = false;
            
            console.log('实时模型连接成功:', result);
            return true;
        } else {
            showToast(`❌ 连接失败: ${result.error}`, 'error');
            updateRealtimeStatus(`连接失败: ${result.error}`, 'error');
            return false;
        }
    } catch (error) {
        console.error('连接实时模型异常:', error);
        showToast('❌ 连接失败，请检查网络', 'error');
        updateRealtimeStatus('连接异常，请检查网络连接', 'error');
        return false;
    }
}

// 断开实时模型连接
async function disconnectRealtimeModel() {
    if (!isRealtimeModel(currentModel)) {
        showToast('当前不是实时语音模型', 'warning');
        return;
    }
    
    try {
        updateRealtimeStatus('正在断开连接...', 'info');
        
        const response = await fetch(`${API_BASE_URL}/api/realtime/disconnect`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ model: currentModel })
        });
        
        const result = await response.json();
        
        if (result.success) {
            showToast(`✅ ${currentModel} 已断开连接`, 'success');
            updateRealtimeStatus('已断开连接', 'success');
            
            // 停止实时响应轮询
            stopRealtimePolling();
            
            // 断开WebRTC连接
            disconnectRealtimeWebRTC();
            
            // 恢复到断开状态的界面
            if (isRealtimeModel(currentModel)) {
                const textInput = document.getElementById('text-input');
                if (textInput) {
                    textInput.placeholder = '选择实时模型后，请先点击上方"连接实时模型"按钮，然后在此输入对话内容...';
                    textInput.style.borderColor = ''; // 恢复默认边框
                    textInput.style.boxShadow = '';
                }
                
                const sendBtn = document.getElementById('send-btn');
                if (sendBtn) {
                    sendBtn.style.backgroundColor = '';
                    sendBtn.title = '';
                }
            }
            
            // 更新按钮状态
            document.getElementById('realtime-connect-btn').disabled = false;
            document.getElementById('realtime-disconnect-btn').disabled = true;
            
            console.log('实时模型断开连接成功:', result);
        } else {
            showToast(`❌ 断开连接失败: ${result.error}`, 'error');
            updateRealtimeStatus(`断开失败: ${result.error}`, 'error');
        }
    } catch (error) {
        console.error('断开实时模型连接异常:', error);
        showToast('❌ 断开连接失败', 'error');
        updateRealtimeStatus('断开连接异常', 'error');
    }
}

// 检查实时模型状态
async function checkRealtimeStatus() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/realtime/status`);
        const result = await response.json();
        
        if (result.success) {
            const models = result.realtime_models;
            let statusText = '实时模型状态:\n';
            
            for (const [modelName, status] of Object.entries(models)) {
                const connected = status.connected ? '🟢 已连接' : '🔴 未连接';
                const sessionId = status.session_id ? ` (${status.session_id})` : '';
                statusText += `• ${modelName}: ${connected}${sessionId}\n`;
                
                // 为OpenAI模型添加特殊说明
                if (modelName === 'gpt-4o-realtime-preview' && !status.connected) {
                    statusText += '  ⚠️ OpenAI API可能需要VPN访问\n';
                }
            }
            
            updateRealtimeStatus(statusText, 'info');
            showToast('状态检查完成', 'success');
            
            // 根据当前模型状态更新按钮
            if (models[currentModel]) {
                const isConnected = models[currentModel].connected;
                document.getElementById('realtime-connect-btn').disabled = isConnected;
                document.getElementById('realtime-disconnect-btn').disabled = !isConnected;
            }
            
            console.log('实时模型状态:', result);
        } else {
            updateRealtimeStatus('状态检查失败', 'error');
            showToast('❌ 状态检查失败', 'error');
        }
    } catch (error) {
        console.error('检查实时模型状态异常:', error);
        updateRealtimeStatus('状态检查异常', 'error');
        showToast('❌ 状态检查失败', 'error');
    }
}

// 更新实时模型状态显示
function updateRealtimeStatus(message, type) {
    const statusElement = document.getElementById('realtime-status');
    if (statusElement) {
        statusElement.textContent = message;
        statusElement.className = `config-status ${type}`;
    }
}

// 发送实时消息
async function sendRealtimeMessage(messageData) {
    try {
        // 检查是否已连接
        const statusResponse = await fetch(`${API_BASE_URL}/api/realtime/status`);
        const statusResult = await statusResponse.json();
        
        if (!statusResult.success) {
            showToast('❌ 无法获取实时模型状态', 'error');
            return;
        }
        
        const modelStatus = statusResult.realtime_models[currentModel];
        if (!modelStatus || !modelStatus.connected) {
            showToast('⚠️ 实时模型未连接，请先连接模型', 'warning');
            
            // 自动尝试连接
            showToast('🔄 正在自动连接实时模型...', 'info');
            const connectSuccess = await connectRealtimeModel();
            if (!connectSuccess) {
                return;
            }
        }
        
        // 准备发送数据
        const sendData = {
            model: currentModel
        };
        
        // 优先发送文字（如果有）
        if (messageData.text) {
            // 首先显示用户消息
            addMessage('user', messageData.text, messageData.images ? 
                messageData.images.map((img, i) => ({data: img, name: `image_${i+1}.png`})) : null);
            
            // 尝试通过WebSocket发送（如果可用）
            if (realtimeWebSocket && realtimeWebSocket.readyState === WebSocket.OPEN) {
                console.log('使用WebSocket发送消息');
                const wsSuccess = sendWebSocketMessage(messageData.text);
                
                if (wsSuccess) {
                    showToast('✅ 消息已通过WebSocket发送', 'success');
                    addSystemMessage('📡 消息已通过WebSocket发送，等待AI实时回复...');
                    return; // WebSocket发送成功，直接返回
                } else {
                    showToast('⚠️ WebSocket发送失败，尝试HTTP方式', 'warning');
                }
            }
            
            // 回退到HTTP方式发送
            sendData.text = messageData.text;
            
            const response = await fetch(`${API_BASE_URL}/api/realtime/send`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(sendData)
            });
            
            const result = await response.json();
            
            if (result.success) {
                // 显示发送成功提示
                showToast('✅ 消息已发送到实时模型（HTTP）', 'success');
                addSystemMessage('📡 消息已通过HTTP发送到实时模型，请等待响应...');
            } else {
                showToast(`❌ 发送失败: ${result.error}`, 'error');
            }
        }
        
        // 如果有音频，发送音频
        if (messageData.audio) {
            const audioData = {
                model: currentModel,
                audio: messageData.audio
            };
            
            const audioResponse = await fetch(`${API_BASE_URL}/api/realtime/send`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(audioData)
            });
            
            const audioResult = await audioResponse.json();
            
            if (audioResult.success) {
                showToast('🎤 音频已发送到实时模型', 'success');
                if (!messageData.text) {
                    addMessage('user', '[语音消息]', null, true);
                    addMessage('assistant', '🎤 语音消息已发送到实时模型，请通过WebSocket监听响应。');
                }
            } else {
                showToast(`❌ 音频发送失败: ${audioResult.error}`, 'error');
            }
        }
        
        // 如果只有图片，提示不支持
        if (!messageData.text && !messageData.audio && messageData.images && messageData.images.length > 0) {
            showToast('⚠️ 实时模型暂不支持纯图片消息', 'warning');
            addMessage('assistant', '⚠️ 实时模型暂不支持纯图片消息，请添加文字描述。');
        }
        
    } catch (error) {
        console.error('发送实时消息失败:', error);
        showToast('❌ 发送实时消息失败', 'error');
    }
}

// 实时模型响应轮询功能
function startRealtimePolling(modelName) {
    if (isRealtimePolling) {
        console.log('实时轮询已在运行');
        return;
    }
    
    isRealtimePolling = true;
    lastResponseTimestamp = Date.now() / 1000; // 当前时间戳
    
    console.log(`开始轮询 ${modelName} 的实时响应...`);
    
    realtimePollingInterval = setInterval(async () => {
        try {
            const response = await fetch(`${API_BASE_URL}/api/realtime/responses?model=${modelName}&since=${lastResponseTimestamp}`);
            const result = await response.json();
            
            if (result.success && result.new_count > 0) {
                console.log(`收到 ${result.new_count} 个新响应`);
                
                // 处理每个响应
                result.responses.forEach(resp => {
                    displayRealtimeResponse(modelName, resp);
                });
                
                // 更新时间戳
                lastResponseTimestamp = result.latest_timestamp;
            }
            
        } catch (error) {
            console.error('轮询实时响应失败:', error);
        }
    }, 1000); // 每秒轮询一次
    
    showToast(`开始监听 ${modelName} 的实时响应`, 'info');
}

function stopRealtimePolling() {
    if (realtimePollingInterval) {
        clearInterval(realtimePollingInterval);
        realtimePollingInterval = null;
    }
    isRealtimePolling = false;
    console.log('停止实时响应轮询');
    showToast('停止监听实时响应', 'info');
}

// WebRTC客户端管理 - 用于OpenAI Realtime API
let realtimePC = null;
let realtimeDataChannel = null;
let localAudioStream = null;
let currentRealtimeModel = null;

async function createRealtimeWebRTC(modelName) {
    if (!isRealtimeModel(modelName)) {
        return false;
    }
    
    // 如果已有连接，先关闭
    if (realtimePC) {
        disconnectRealtimeWebRTC();
    }
    
    try {
        console.log(`创建WebRTC连接: ${modelName}`);
        currentRealtimeModel = modelName;
        
        // 1. 获取用户麦克风
        console.log('获取麦克风权限...');
        localAudioStream = await navigator.mediaDevices.getUserMedia({
            audio: {
                echoCancellation: true,
                noiseSuppression: true,
                autoGainControl: true
            },
            video: false
        });
        
        // 2. 创建session获取ephemeral key (仅对GPT-4o-Realtime)
        if (modelName === 'gpt-4o-realtime-preview') {
            console.log('创建OpenAI Realtime session...');
            const sessionResponse = await fetch(`${API_BASE_URL}/api/realtime/session`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    voice: 'alloy',
                    instructions: '你是一个有用的AI助手，请用中文回复用户的问题。'
                })
            });
            
            if (!sessionResponse.ok) {
                const error = await sessionResponse.text();
                throw new Error(`Session creation failed: ${sessionResponse.status} ${error}`);
            }
            
            const session = await sessionResponse.json();
            const ephemeralKey = session.client_secret?.value;
            
            if (!ephemeralKey) {
                throw new Error('No ephemeral key received from session');
            }
            
            console.log('Session创建成功，设置WebRTC连接...');
            
            // 3. 创建RTCPeerConnection
            realtimePC = new RTCPeerConnection();
            
            // 添加音频轨道
            const [audioTrack] = localAudioStream.getAudioTracks();
            realtimePC.addTrack(audioTrack, localAudioStream);
            
            // 处理远程音频
            realtimePC.ontrack = (event) => {
                console.log('收到远程音频流');
                const remoteAudio = document.createElement('audio');
                remoteAudio.srcObject = event.streams[0];
                remoteAudio.autoplay = true;
                remoteAudio.playsInline = true;
                remoteAudio.style.display = 'none';
                document.body.appendChild(remoteAudio);
                
                // 尝试播放
                remoteAudio.play().catch(e => {
                    console.log('音频自动播放被阻止，需要用户交互');
                });
            };
            
            // 4. 创建数据通道
            realtimeDataChannel = realtimePC.createDataChannel('oai-events');
            
            realtimeDataChannel.onopen = () => {
                console.log('WebRTC数据通道已打开');
                
                // 配置session
                const sessionConfig = {
                    type: "session.update",
                    session: {
                        modalities: ["text", "audio"],
                        instructions: "你是一个有用的AI助手，请用中文回复用户的问题。当用户说话时，请自然地回应。",
                        voice: "alloy",
                        input_audio_format: "pcm16",
                        output_audio_format: "pcm16",
                        turn_detection: {
                            type: "server_vad",
                            threshold: 0.6,
                            prefix_padding_ms: 300,
                            silence_duration_ms: 800,
                            create_response: true
                        },
                        temperature: 0.8,
                        max_response_output_tokens: "inf"
                    }
                };
                
                realtimeDataChannel.send(JSON.stringify(sessionConfig));
                console.log('Session配置已发送');
                
                showToast('✅ WebRTC连接成功！', 'success');
                addSystemMessage('📡 WebRTC已连接，现在可以直接说话进行实时对话！');
            };
            
            realtimeDataChannel.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    console.log('WebRTC收到消息:', data.type || 'unknown');
                    displayWebRTCResponse(data);
                } catch (error) {
                    console.error('解析WebRTC消息失败:', error);
                }
            };
            
            realtimeDataChannel.onerror = (error) => {
                console.error('WebRTC数据通道错误:', error);
                showToast('❌ WebRTC数据通道错误', 'error');
            };
            
            realtimeDataChannel.onclose = () => {
                console.log('WebRTC数据通道已关闭');
                showToast('🔌 WebRTC连接已断开', 'warning');
            };
            
            // 5. 创建offer并设置本地描述
            const offer = await realtimePC.createOffer({
                offerToReceiveAudio: true,
                offerToReceiveVideo: false
            });
            await realtimePC.setLocalDescription(offer);
            
            // 等待ICE收集完成
            await waitForIceGathering(realtimePC);
            
            // 6. 通过代理发送SDP到OpenAI
            console.log('发送SDP offer到OpenAI...');
            const sdpResponse = await fetch(`${API_BASE_URL}/api/realtime/sdp`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    sdp: realtimePC.localDescription.sdp,
                    ephemeralKey: ephemeralKey
                })
            });
            
            if (!sdpResponse.ok) {
                const error = await sdpResponse.text();
                throw new Error(`SDP exchange failed: ${sdpResponse.status} ${error}`);
            }
            
            const answerSDP = await sdpResponse.text();
            const answer = { type: "answer", sdp: answerSDP };
            await realtimePC.setRemoteDescription(answer);
            
            // 监听连接状态
            realtimePC.onconnectionstatechange = () => {
                console.log(`WebRTC连接状态: ${realtimePC.connectionState}`);
                if (realtimePC.connectionState === 'connected') {
                    console.log('✅ WebRTC连接已建立');
                } else if (realtimePC.connectionState === 'failed') {
                    showToast('❌ WebRTC连接失败', 'error');
                    disconnectRealtimeWebRTC();
                }
            };
            
            return true;
        } else {
            // 对于其他实时模型(如GLM)，仍使用WebSocket
            return await createRealtimeWebSocket(modelName);
        }
        
    } catch (error) {
        console.error('WebRTC连接失败:', error);
        showToast(`❌ WebRTC连接失败: ${error.message}`, 'error');
        disconnectRealtimeWebRTC();
        return false;
    }
}

// 等待ICE收集完成
function waitForIceGathering(pc) {
    return new Promise((resolve) => {
        if (pc.iceGatheringState === 'complete') {
            resolve();
        } else {
            pc.addEventListener('icegatheringstatechange', () => {
                if (pc.iceGatheringState === 'complete') {
                    resolve();
                }
            });
        }
    });
}

function disconnectRealtimeWebRTC() {
    console.log('断开WebRTC连接...');
    
    if (realtimeDataChannel) {
        realtimeDataChannel.close();
        realtimeDataChannel = null;
    }
    
    if (realtimePC) {
        realtimePC.close();
        realtimePC = null;
    }
    
    if (localAudioStream) {
        localAudioStream.getTracks().forEach(track => track.stop());
        localAudioStream = null;
    }
    
    // 移除音频元素
    document.querySelectorAll('audio').forEach(audio => {
        if (audio.srcObject) {
            audio.remove();
        }
    });
    
    currentRealtimeModel = null;
    showToast('🔌 WebRTC连接已断开', 'info');
}

// 兼容GLM等其他实时模型的WebSocket连接
async function createRealtimeWebSocket(modelName) {
    try {
        // 连接到我们后端的WebSocket代理
        const wsUrl = `ws://localhost:8765/realtime-proxy?model=${modelName}`;
        console.log(`尝试连接WebSocket: ${wsUrl}`);
        
        const realtimeWebSocket = new WebSocket(wsUrl);
        
        return new Promise((resolve, reject) => {
            realtimeWebSocket.onopen = () => {
                console.log(`WebSocket连接已建立: ${modelName}`);
                showToast(`🔗 ${modelName} WebSocket连接成功`, 'success');
                addSystemMessage(`🔗 ${modelName} WebSocket已连接，现在可以进行实时对话！`);
                resolve(true);
            };
            
            realtimeWebSocket.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    console.log('收到WebSocket消息:', data);
                    displayWebSocketResponse(data);
                } catch (error) {
                    console.error('解析WebSocket消息失败:', error);
                }
            };
            
            realtimeWebSocket.onclose = (event) => {
                console.log(`WebSocket连接已关闭: ${modelName}`, event.code, event.reason);
                showToast(`🔌 ${modelName} WebSocket连接已断开`, 'warning');
            };
            
            realtimeWebSocket.onerror = (error) => {
                console.error('WebSocket错误:', error);
                showToast(`❌ ${modelName} WebSocket连接错误`, 'error');
                reject(error);
            };
        });
        
    } catch (error) {
        console.error('创建WebSocket连接失败:', error);
        showToast(`❌ 无法创建WebSocket连接: ${error.message}`, 'error');
        return false;
    }
}

// 显示WebRTC实时响应
function displayWebRTCResponse(data) {
    const chatMessages = document.getElementById('chat-messages');
    if (!chatMessages) return;
    
    switch (data.type) {
        case "session.created":
            console.log("✅ WebRTC Session创建成功");
            break;
        case "session.updated":
            console.log("✅ WebRTC Session配置更新");
            break;
        case "input_audio_buffer.speech_started":
            console.log("🎤 检测到用户开始说话");
            addSystemMessage("🎤 检测到语音输入...");
            break;
        case "input_audio_buffer.speech_stopped":
            console.log("⏹️ 用户停止说话");
            break;
        case "conversation.item.input_audio_transcription.completed":
            if (data.transcript) {
                console.log("👤 用户说:", data.transcript);
                addMessage('user', data.transcript, null);
            }
            break;
        case "response.created":
            console.log("🤖 AI开始生成回复");
            break;
        case "response.audio_transcript.delta":
            // 处理AI回复的文本流
            if (data.delta) {
                updateLatestAIMessage(data.delta);
            }
            break;
        case "response.audio_transcript.done":
            if (data.transcript) {
                console.log("📄 AI完整回复:", data.transcript);
                finalizeLatestAIMessage(data.transcript);
            }
            break;
        case "response.audio.delta":
            console.log("🔊 收到音频数据块");
            break;
        case "response.audio.done":
            console.log("✅ 音频回复完成");
            break;
        case "response.done":
            console.log("✅ AI回复完成");
            break;
        case "error":
            console.error("❌ WebRTC错误:", data.error);
            showToast(`WebRTC错误: ${data.error?.message || 'Unknown error'}`, 'error');
            break;
        default:
            console.log(`ℹ️ WebRTC事件: ${data.type}`);
    }
}

// AI消息流式更新辅助函数
let latestAIMessageElement = null;

function updateLatestAIMessage(delta) {
    if (!latestAIMessageElement) {
        // 创建新的AI消息元素
        latestAIMessageElement = addMessage('assistant', '', null, true); // 第四个参数表示流式消息
    }
    
    // 更新消息内容
    const messageContent = latestAIMessageElement.querySelector('.message-content');
    if (messageContent) {
        messageContent.textContent += delta;
    }
}

function finalizeLatestAIMessage(fullTranscript) {
    if (latestAIMessageElement) {
        const messageContent = latestAIMessageElement.querySelector('.message-content');
        if (messageContent) {
            messageContent.textContent = fullTranscript; // 确保显示完整文本
        }
        latestAIMessageElement = null; // 重置
    }
}

// 显示WebSocket实时响应
function displayWebSocketResponse(data) {
    const chatMessages = document.getElementById('chat-messages');
    if (!chatMessages) return;
    
    // 创建响应消息元素
    const messageDiv = document.createElement('div');
    messageDiv.className = 'message assistant-message realtime-websocket-response';
    
    let content = '';
    let audioContent = '';
    
    // 处理不同类型的响应
    switch (data.type) {
        case 'response.text.delta':
        case 'response.audio_transcript.delta':
            content = `<div class="realtime-text-delta">📝 ${escapeHtml(data.delta || data.transcript || '')}</div>`;
            break;
            
        case 'response.text.done':
        case 'response.audio_transcript.done':
            content = `<div class="realtime-text-complete">✅ ${escapeHtml(data.text || data.transcript || '')}</div>`;
            break;
            
        case 'response.audio.delta':
            content = `<div class="realtime-audio-delta">🔊 音频数据接收中...</div>`;
            break;
            
        case 'response.audio.done':
            if (data.audio) {
                const audioId = `audio_${Date.now()}`;
                audioContent = `
                    <div class="realtime-audio-complete">
                        <p>🎵 AI语音回复：</p>
                        <audio id="${audioId}" controls>
                            <source src="data:audio/wav;base64,${data.audio}" type="audio/wav">
                            您的浏览器不支持音频播放。
                        </audio>
                    </div>
                `;
            }
            break;
            
        case 'session.created':
            content = `<div class="realtime-info">🟢 会话已创建</div>`;
            break;
            
        case 'session.updated':
            content = `<div class="realtime-info">🔄 会话已更新</div>`;
            break;
            
        case 'error':
            content = `<div class="realtime-error">❌ 错误: ${escapeHtml(JSON.stringify(data.error))}</div>`;
            break;
            
        default:
            content = `<div class="realtime-debug">🔧 ${data.type}</div>`;
    }
    
    messageDiv.innerHTML = `
        <div class="message-header">
            <span class="sender">🤖 AI (实时WebSocket)</span>
            <span class="timestamp">${new Date().toLocaleTimeString()}</span>
        </div>
        <div class="message-content">
            ${content}
            ${audioContent}
        </div>
    `;
    
    chatMessages.appendChild(messageDiv);
    chatMessages.scrollTop = chatMessages.scrollHeight;
    
    // 如果有音频，尝试自动播放
    if (audioContent) {
        setTimeout(() => {
            const audioElement = messageDiv.querySelector('audio');
            if (audioElement) {
                audioElement.play().catch(e => console.log('自动播放失败:', e));
            }
        }, 100);
    }
}

// 通过WebSocket发送消息
function sendWebSocketMessage(text) {
    if (!realtimeWebSocket || realtimeWebSocket.readyState !== WebSocket.OPEN) {
        showToast('❌ WebSocket未连接', 'error');
        return false;
    }
    
    try {
        // 发送文本消息
        const message = {
            type: 'conversation.item.create',
            item: {
                type: 'message',
                role: 'user',
                content: [{
                    type: 'input_text',
                    text: text
                }]
            }
        };
        
        realtimeWebSocket.send(JSON.stringify(message));
        
        // 请求响应
        const responseRequest = {
            type: 'response.create',
            response: {
                modalities: ['text', 'audio']
            }
        };
        
        realtimeWebSocket.send(JSON.stringify(responseRequest));
        
        console.log('WebSocket消息已发送:', text);
        return true;
    } catch (error) {
        console.error('WebSocket发送失败:', error);
        showToast(`❌ WebSocket发送失败: ${error.message}`, 'error');
        return false;
    }
}

function displayRealtimeResponse(modelName, response) {
    const chatContainer = document.getElementById('chat-container');
    const messageDiv = document.createElement('div');
    messageDiv.className = 'message ai-message realtime-response';
    
    let content = '';
    let hasContent = false;
    
    // 根据响应类型显示内容
    if (response.content_type === 'text_delta' || response.content_type === 'transcript_delta') {
        if (response.text && response.text.trim()) {
            content += `<div class="realtime-text-delta">${escapeHtml(response.text)}</div>`;
            hasContent = true;
        }
    } else if (response.content_type === 'text_complete' || response.content_type === 'transcript_complete') {
        if (response.text && response.text.trim()) {
            content += `<div class="realtime-text-complete">${escapeHtml(response.text)}</div>`;
            hasContent = true;
        }
    } else if (response.content_type === 'audio_delta') {
        content += `<div class="realtime-audio-info">🎵 音频数据片段 (${response.audio ? response.audio.length : 0} 字节)</div>`;
        hasContent = true;
    } else if (response.content_type === 'audio_complete') {
        if (response.audio) {
            content += `<div class="realtime-audio-complete">
                <div class="audio-info">🎵 完整音频响应</div>
                <audio controls>
                    <source src="data:audio/wav;base64,${response.audio}" type="audio/wav">
                    您的浏览器不支持音频播放。
                </audio>
            </div>`;
            hasContent = true;
        }
    } else {
        // 显示原始响应信息，但只有在真正有意义的数据时
        if (response.type && !response.type.includes('error') && !response.type.includes('session')) {
            content += `<div class="realtime-debug">
                <strong>📡 实时响应:</strong> ${response.type}<br>
                <strong>⏰ 时间:</strong> ${new Date(response.timestamp * 1000).toLocaleTimeString()}
            </div>`;
            hasContent = true;
        }
    }
    
    if (hasContent) {
        messageDiv.innerHTML = `
            <div class="message-header">
                <strong>🤖 ${modelName} (实时)</strong>
                <span class="timestamp">${new Date(response.timestamp * 1000).toLocaleTimeString()}</span>
            </div>
            <div class="message-content">${content}</div>
        `;
        
        chatContainer.appendChild(messageDiv);
        chatContainer.scrollTop = chatContainer.scrollHeight;
        
        // 添加特殊样式
        if (response.content_type.includes('complete')) {
            messageDiv.classList.add('complete-response');
        } else if (response.content_type.includes('delta')) {
            messageDiv.classList.add('delta-response');
        }
        
        console.log(`显示实时响应: ${response.type} (${response.content_type})`);
    }
}

function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

function updateRealtimeButtons(connected) {
    const connectBtn = document.querySelector('#realtime-controls button:nth-child(1)');
    const disconnectBtn = document.querySelector('#realtime-controls button:nth-child(2)');
    
    if (connectBtn) {
        connectBtn.disabled = connected;
        connectBtn.textContent = connected ? '已连接' : '连接实时模型';
    }
    
    if (disconnectBtn) {
        disconnectBtn.disabled = !connected;
        disconnectBtn.textContent = connected ? '断开连接' : '未连接';
    }
}

// 页面加载完成后初始化记忆
window.addEventListener('DOMContentLoaded', function() {
    // 延迟初始化，确保其他DOM元素已加载
    setTimeout(() => {
        renderMemories();
    }, 100);
});

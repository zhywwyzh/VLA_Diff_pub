package main

import (
	"encoding/json"
	"fmt"
	"io/ioutil"
	"log"
	"net/http"
	"os"
	"strings"
	"time"
)

// Message 消息结构体
type Message struct {
	ID         string    `json:"id"`
	Text       string    `json:"text"`
	Source     string    `json:"source"`
	Timestamp  time.Time `json:"timestamp"`
	ReceivedAt time.Time `json:"received_at"`
	Length     int       `json:"length"`
}

// ReceiveRequest 接收请求结构体
type ReceiveRequest struct {
	Text      string `json:"text"`
	Source    string `json:"source"`
	Timestamp string `json:"timestamp"`
}

// Response 响应结构体
type Response struct {
	Success bool        `json:"success"`
	Message string      `json:"message,omitempty"`
	Error   string      `json:"error,omitempty"`
	Data    interface{} `json:"data,omitempty"`
}

var receivedMessages []Message

// processText 处理接收到的文字内容
func processText(text, source string) {
	fmt.Println("\n🔥 开始处理文字内容...")
	fmt.Printf("📝 内容: %s\n", text)
	fmt.Printf("📊 长度: %d 字符\n", len([]rune(text)))
	fmt.Printf("🕒 时间: %s\n", time.Now().Format("2006-01-02 15:04:05"))

	// ========================================
	// 在这里添加您的处理逻辑
	// ========================================

	// 示例1: 文字分析
	if len([]rune(text)) > 50 {
		fmt.Println("📏 这是一条长消息")
	} else {
		fmt.Println("📏 这是一条短消息")
	}

	// 示例2: 关键词检测
	keywords := []string{"测试", "帮助", "问题", "API", "Go"}
	var foundKeywords []string
	for _, keyword := range keywords {
		if strings.Contains(text, keyword) {
			foundKeywords = append(foundKeywords, keyword)
		}
	}
	if len(foundKeywords) > 0 {
		fmt.Printf("🔍 发现关键词: %s\n", strings.Join(foundKeywords, ", "))
	}

	// 示例3: 保存到文件
	err := saveToFile(text)
	if err != nil {
		fmt.Printf("❌ 保存失败: %v\n", err)
	} else {
		fmt.Println("💾 已保存到文件")
	}

	// 示例4: 发送到其他系统
	// sendToOtherSystem(text)

	fmt.Println("✅ 处理完成\n")
}

// saveToFile 保存文字到文件
func saveToFile(text string) error {
	logData := fmt.Sprintf("[%s] %s\n", time.Now().Format("2006-01-02 15:04:05"), text)
	return ioutil.WriteFile("go_received_texts.txt", []byte(logData), 0644)
}

// sendToOtherSystem 发送到其他系统的示例函数
func sendToOtherSystem(text string) {
	// 示例：发送到其他API
	/*
	data := map[string]string{"text": text}
	jsonData, _ := json.Marshal(data)
	
	resp, err := http.Post("http://your-other-system.com/api", 
		"application/json", 
		strings.NewReader(string(jsonData)))
	
	if err != nil {
		fmt.Printf("发送到其他系统失败: %v\n", err)
		return
	}
	defer resp.Body.Close()
	
	if resp.StatusCode == 200 {
		fmt.Println("发送到其他系统成功")
	} else {
		fmt.Printf("发送到其他系统失败: HTTP %d\n", resp.StatusCode)
	}
	*/
}

// receiveHandler 接收文字消息的处理器
func receiveHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.Header().Set("Access-Control-Allow-Origin", "*")
	w.Header().Set("Access-Control-Allow-Methods", "POST, OPTIONS")
	w.Header().Set("Access-Control-Allow-Headers", "Content-Type")

	if r.Method == "OPTIONS" {
		w.WriteHeader(http.StatusOK)
		return
	}

	if r.Method != "POST" {
		w.WriteHeader(http.StatusMethodNotAllowed)
		json.NewEncoder(w).Encode(Response{
			Success: false,
			Error:   "只支持POST方法",
		})
		return
	}

	var req ReceiveRequest
	err := json.NewDecoder(r.Body).Decode(&req)
	if err != nil {
		w.WriteHeader(http.StatusBadRequest)
		json.NewEncoder(w).Encode(Response{
			Success: false,
			Error:   "JSON解析失败",
		})
		return
	}

	if req.Text == "" {
		w.WriteHeader(http.StatusBadRequest)
		json.NewEncoder(w).Encode(Response{
			Success: false,
			Error:   "文字内容为空",
		})
		return
	}

	// 创建消息记录
	message := Message{
		ID:         fmt.Sprintf("%d", time.Now().UnixNano()),
		Text:       req.Text,
		Source:     req.Source,
		Timestamp:  time.Now(),
		ReceivedAt: time.Now(),
		Length:     len([]rune(req.Text)),
	}

	if req.Source == "" {
		message.Source = "unknown"
	}

	// 保存消息
	receivedMessages = append(receivedMessages, message)

	// 异步处理文字，避免阻塞响应
	go processText(req.Text, req.Source)

	// 返回响应
	json.NewEncoder(w).Encode(Response{
		Success: true,
		Message: "文字已接收并开始处理",
		Data: map[string]interface{}{
			"id":     message.ID,
			"length": message.Length,
		},
	})
}

// messagesHandler 获取所有消息的处理器
func messagesHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")

	json.NewEncoder(w).Encode(Response{
		Success: true,
		Data: map[string]interface{}{
			"total":    len(receivedMessages),
			"messages": receivedMessages,
		},
	})
}

// healthHandler 健康检查处理器
func healthHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")

	json.NewEncoder(w).Encode(Response{
		Success: true,
		Data: map[string]interface{}{
			"status":        "running",
			"service":       "go-text-listener",
			"message_count": len(receivedMessages),
			"time":          time.Now(),
		},
	})
}

// indexHandler 主页处理器
func indexHandler(w http.ResponseWriter, r *http.Request) {
	html := fmt.Sprintf(`
		<h1>🎧 Go 文字监听器</h1>
		<p><strong>状态:</strong> 运行中</p>
		<p><strong>接收端点:</strong> /api/receive</p>
		<p><strong>消息数量:</strong> %d</p>
		<p><strong>时间:</strong> %s</p>
		<p><strong>说明:</strong> 这个Go服务会接收并处理转发的文字内容</p>
	`, len(receivedMessages), time.Now().Format("2006-01-02 15:04:05"))

	w.Header().Set("Content-Type", "text/html; charset=utf-8")
	w.Write([]byte(html))
}

func main() {
	port := "8000"

	// 设置路由
	http.HandleFunc("/api/receive", receiveHandler)
	http.HandleFunc("/api/messages", messagesHandler)
	http.HandleFunc("/api/health", healthHandler)
	http.HandleFunc("/", indexHandler)

	fmt.Println("🎧 启动Go文字监听器...")
	fmt.Printf("📍 服务地址: http://localhost:%s\n", port)
	fmt.Printf("📍 接收端点: http://localhost:%s/api/receive\n", port)
	fmt.Printf("💡 在Gemini聊天系统中设置转发URL为: http://localhost:%s/api/receive\n", port)
	fmt.Println("🔧 请在 processText() 函数中添加您的处理逻辑")
	fmt.Println(strings.Repeat("-", 60))

	// 启动服务器
	log.Fatal(http.ListenAndServe(":"+port, nil))
}

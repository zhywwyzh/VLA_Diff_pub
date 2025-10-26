/**
 * Java 版本的文字监听器
 * 接收从Gemini聊天系统转发的文字内容
 * 
 * 编译: javac -cp ".:gson-2.8.9.jar" JavaTextListener.java
 * 运行: java -cp ".:gson-2.8.9.jar" JavaTextListener
 */

import com.sun.net.httpserver.HttpServer;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpExchange;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import java.io.*;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.*;
import java.util.concurrent.Executors;

public class JavaTextListener {
    
    private static final int PORT = 9000;
    private static final Gson gson = new GsonBuilder()
            .setPrettyPrinting()
            .setDateFormat("yyyy-MM-dd'T'HH:mm:ss")
            .create();
    
    private static final List<Message> receivedMessages = new ArrayList<>();
    
    // 消息类
    static class Message {
        public String id;
        public String text;
        public String source;
        public String timestamp;
        public String receivedAt;
        public int length;
        
        public Message(String text, String source) {
            this.id = String.valueOf(System.currentTimeMillis());
            this.text = text;
            this.source = source != null ? source : "unknown";
            this.timestamp = LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME);
            this.receivedAt = LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME);
            this.length = text.length();
        }
    }
    
    // 响应类
    static class Response {
        public boolean success;
        public String message;
        public String error;
        public Object data;
        
        public Response(boolean success, String message) {
            this.success = success;
            this.message = message;
        }
        
        public Response(boolean success, String message, Object data) {
            this.success = success;
            this.message = message;
            this.data = data;
        }
        
        public static Response error(String error) {
            Response response = new Response(false, null);
            response.error = error;
            return response;
        }
    }
    
    /**
     * 处理接收到的文字内容
     */
    private static void processText(String text, String source) {
        System.out.println("\n🔥 开始处理文字内容...");
        System.out.println("📝 内容: " + text);
        System.out.println("📊 长度: " + text.length() + " 字符");
        System.out.println("🕒 时间: " + LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss")));
        
        // ========================================
        // 在这里添加您的处理逻辑
        // ========================================
        
        // 示例1: 文字分析
        if (text.length() > 50) {
            System.out.println("📏 这是一条长消息");
        } else {
            System.out.println("📏 这是一条短消息");
        }
        
        // 示例2: 关键词检测
        String[] keywords = {"测试", "帮助", "问题", "API", "Java"};
        List<String> foundKeywords = new ArrayList<>();
        for (String keyword : keywords) {
            if (text.contains(keyword)) {
                foundKeywords.add(keyword);
            }
        }
        if (!foundKeywords.isEmpty()) {
            System.out.println("🔍 发现关键词: " + String.join(", ", foundKeywords));
        }
        
        // 示例3: 保存到文件
        try {
            saveToFile(text);
            System.out.println("💾 已保存到文件");
        } catch (IOException e) {
            System.out.println("❌ 保存失败: " + e.getMessage());
        }
        
        // 示例4: 发送到其他系统
        // sendToOtherSystem(text);
        
        System.out.println("✅ 处理完成\n");
    }
    
    /**
     * 保存文字到文件
     */
    private static void saveToFile(String text) throws IOException {
        String logData = "[" + LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss")) + "] " + text + "\n";
        Files.write(Paths.get("java_received_texts.txt"), logData.getBytes(StandardCharsets.UTF_8), 
                   StandardOpenOption.CREATE, StandardOpenOption.APPEND);
    }
    
    /**
     * 发送到其他系统的示例函数
     */
    private static void sendToOtherSystem(String text) {
        // 示例：使用HttpURLConnection发送到其他API
        /*
        try {
            URL url = new URL("http://your-other-system.com/api");
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("POST");
            conn.setRequestProperty("Content-Type", "application/json");
            conn.setDoOutput(true);
            
            String jsonData = gson.toJson(Collections.singletonMap("text", text));
            
            try (OutputStream os = conn.getOutputStream()) {
                os.write(jsonData.getBytes(StandardCharsets.UTF_8));
            }
            
            int responseCode = conn.getResponseCode();
            if (responseCode == 200) {
                System.out.println("发送到其他系统成功");
            } else {
                System.out.println("发送到其他系统失败: HTTP " + responseCode);
            }
        } catch (Exception e) {
            System.out.println("发送到其他系统失败: " + e.getMessage());
        }
        */
    }
    
    // 接收文字消息的处理器
    static class ReceiveHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            setCORSHeaders(exchange);
            
            if ("OPTIONS".equals(exchange.getRequestMethod())) {
                exchange.sendResponseHeaders(200, 0);
                exchange.close();
                return;
            }
            
            if (!"POST".equals(exchange.getRequestMethod())) {
                sendResponse(exchange, 405, Response.error("只支持POST方法"));
                return;
            }
            
            try {
                // 读取请求体
                String requestBody = readRequestBody(exchange);
                Map<String, Object> requestData = gson.fromJson(requestBody, Map.class);
                
                String text = (String) requestData.get("text");
                if (text == null || text.trim().isEmpty()) {
                    sendResponse(exchange, 400, Response.error("文字内容为空"));
                    return;
                }
                
                String source = (String) requestData.get("source");
                
                // 创建消息记录
                Message message = new Message(text, source);
                receivedMessages.add(message);
                
                // 异步处理文字，避免阻塞响应
                Executors.newSingleThreadExecutor().submit(() -> processText(text, source));
                
                // 返回响应
                Map<String, Object> responseData = new HashMap<>();
                responseData.put("id", message.id);
                responseData.put("length", message.length);
                
                sendResponse(exchange, 200, new Response(true, "文字已接收并开始处理", responseData));
                
            } catch (Exception e) {
                sendResponse(exchange, 500, Response.error("处理请求时出错: " + e.getMessage()));
            }
        }
    }
    
    // 获取所有消息的处理器
    static class MessagesHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            setCORSHeaders(exchange);
            
            Map<String, Object> data = new HashMap<>();
            data.put("total", receivedMessages.size());
            data.put("messages", receivedMessages);
            
            sendResponse(exchange, 200, new Response(true, null, data));
        }
    }
    
    // 健康检查处理器
    static class HealthHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            setCORSHeaders(exchange);
            
            Map<String, Object> data = new HashMap<>();
            data.put("status", "running");
            data.put("service", "java-text-listener");
            data.put("message_count", receivedMessages.size());
            data.put("time", LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME));
            
            sendResponse(exchange, 200, new Response(true, null, data));
        }
    }
    
    // 主页处理器
    static class IndexHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            String html = String.format(
                "<h1>🎧 Java 文字监听器</h1>" +
                "<p><strong>状态:</strong> 运行中</p>" +
                "<p><strong>接收端点:</strong> /api/receive</p>" +
                "<p><strong>消息数量:</strong> %d</p>" +
                "<p><strong>时间:</strong> %s</p>" +
                "<p><strong>说明:</strong> 这个Java服务会接收并处理转发的文字内容</p>",
                receivedMessages.size(),
                LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss"))
            );
            
            exchange.getResponseHeaders().set("Content-Type", "text/html; charset=utf-8");
            exchange.sendResponseHeaders(200, html.getBytes(StandardCharsets.UTF_8).length);
            
            try (OutputStream os = exchange.getResponseBody()) {
                os.write(html.getBytes(StandardCharsets.UTF_8));
            }
        }
    }
    
    // 工具方法
    private static void setCORSHeaders(HttpExchange exchange) {
        exchange.getResponseHeaders().set("Access-Control-Allow-Origin", "*");
        exchange.getResponseHeaders().set("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
        exchange.getResponseHeaders().set("Access-Control-Allow-Headers", "Content-Type");
        exchange.getResponseHeaders().set("Content-Type", "application/json; charset=utf-8");
    }
    
    private static String readRequestBody(HttpExchange exchange) throws IOException {
        try (BufferedReader reader = new BufferedReader(
                new InputStreamReader(exchange.getRequestBody(), StandardCharsets.UTF_8))) {
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                sb.append(line);
            }
            return sb.toString();
        }
    }
    
    private static void sendResponse(HttpExchange exchange, int statusCode, Response response) throws IOException {
        String jsonResponse = gson.toJson(response);
        exchange.sendResponseHeaders(statusCode, jsonResponse.getBytes(StandardCharsets.UTF_8).length);
        
        try (OutputStream os = exchange.getResponseBody()) {
            os.write(jsonResponse.getBytes(StandardCharsets.UTF_8));
        }
    }
    
    public static void main(String[] args) throws IOException {
        HttpServer server = HttpServer.create(new InetSocketAddress(PORT), 0);
        
        // 设置路由
        server.createContext("/api/receive", new ReceiveHandler());
        server.createContext("/api/messages", new MessagesHandler());
        server.createContext("/api/health", new HealthHandler());
        server.createContext("/", new IndexHandler());
        
        server.setExecutor(Executors.newFixedThreadPool(10));
        server.start();
        
        System.out.println("🎧 启动Java文字监听器...");
        System.out.println("📍 服务地址: http://localhost:" + PORT);
        System.out.println("📍 接收端点: http://localhost:" + PORT + "/api/receive");
        System.out.println("💡 在Gemini聊天系统中设置转发URL为: http://localhost:" + PORT + "/api/receive");
        System.out.println("🔧 请在 processText() 方法中添加您的处理逻辑");
        System.out.println("-".repeat(60));
        
        // 优雅关闭
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            System.out.println("\n🛑 正在关闭服务器...");
            server.stop(0);
        }));
    }
}

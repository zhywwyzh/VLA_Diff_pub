from client_example import GeminiMessageClient

client = GeminiMessageClient()

while True:
    input("Press Enter to receive a message...")
    message = client.get_all_messages()
    if message:
        print(f"Received message: {message}")
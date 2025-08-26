import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'pipeline'))
from client_example import GeminiMessageClient

client = GeminiMessageClient()
message = None

while True:
    input("Press Enter to receive a message...")
    message = client.get_all_messages()
    if message:
        print(f"Received message: {message}")
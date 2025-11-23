
import socket
import time

UDP_PORT = 50000

def listen():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("", UDP_PORT))
    
    print("=" * 50)
    print(f"UDP Listener started on port {UDP_PORT}")
    print("Waiting for Pico to send data...")
    print("Press Ctrl+C to stop")
    print("=" * 50)
    
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            timestamp = time.strftime('%H:%M:%S')
            message = data.decode('utf-8')
            
            print(f"\n[{timestamp}] Received from {addr[0]}:{addr[1]}")
            print(f"Raw data: {repr(message)}")
            print(f"Message: {message.strip()}")
            print("-" * 50)
    except KeyboardInterrupt:
        print("\n\nListener stopped.")
    finally:
        sock.close()

if __name__ == "__main__":
    listen()
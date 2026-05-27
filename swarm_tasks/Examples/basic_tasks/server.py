import socket

class SocketServer:
    def __init__(self, ip, port,isBlocking=True):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.sock.setblocking(isBlocking)
        print(f"Socket server started on {ip}:{port}")
        
    def monitor(self):
        try:
            data, addr = self.sock.recvfrom(1024)  # Buffer size is 1024 bytes
            return data.decode(), addr
        except socket.error as e:
            print(f"Socket error: {e}")
            return None, None
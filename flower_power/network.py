import socket

class Server:
    def __init__(self, ip_address, port):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((ip_address, port))
        print('Waiting for client connection...')
        self.server_socket.listen(1)
        self.client_socket, self.client_address = self.server_socket.accept()
        print(f'Initialized server on {ip_address}:{port}')

    def send_to_client(self, packet: str):
        self.client_socket.send(packet.encode())
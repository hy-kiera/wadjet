import socket
import json

def _send(socket, send_data):
    json_data = json.JSONEncoder().encode(send_data)
    socket.sendall(json_data.encode()) 


def _recv(socket):
    recv_data = socket.recv(4096)
    json_data = json.loads(recv_data.decode())

    return json_data

class Server(object):
    backlog =1
    client =None


    def __init__(self, host, port):
        self.socket = socket.socket()
        self.socket.bind((host, port))
        self.socket.listen(self.backlog)

    def __del__(self):
        self.close()

    def accept(self):
        if self.client:
            self.client.close()
        
        self.client, self.client_addr =self.socket.accept()
        return self

    def send(self, data):
        if not self.client:
            raise Exception('Cantnot send data, no client is connected.')

        _send(self.client, data)
        return self

    def recv(self):
        if not self.client:
            raise Exception('Cannot receive data, no client is connected.')
        
        return _recv(self.client)

    def close(self):
        if self.client:
            self.client.close()
            self.client =None
                
        if self.socket:
            self.socket.close()
            self.socket =None


class Client(object):
    socket =None

    def __del__(self):
        self.close()

    def connect(self, host, port):
        self.socket = socket.socket()
        self.socket.connect((host, port))
        print("self.socket 2 : ", self.socket)
        return self

    def send(self, data):
        if not self.socket:
            raise Exception('You have to connect first before sending data.')        	
            
        _send(self.socket, data)
        return self
    
    def recv(self):
        if not self.socket:
            raise Exception('You have to connect first before receving data.')
            
        return _recv(self.socket)

    def close(self):
        if self.socket:
            self.socket.close()
            self.socket =None

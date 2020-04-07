import socket
import json
import sys
import time
import msgpack
import struct

BUFSIZE = 4096

def _send(socket, send_data):
    json_data = msgpack.packb(send_data, use_bin_type=True)
    
    print("=" * 50)
    print("Send {0} bytes of json data.".format(sys.getsizeof(json_data)))
    
    socket.sendall(struct.pack('>I', len(json_data)) + json_data)


def _recv(socket):
    raw_msglen = _recv_all(socket, 4)
    if not raw_msglen:
        return None

    msglen = struct.unpack('>I', raw_msglen)[0]
    recv_data = _recv_all(socket, msglen)
    
    print("=" * 50)
    print("Received {0} bytes of data.".format(sys.getsizeof(recv_data)))
    return recv_data

def _recv_all(socket, n):
    data = bytearray()
    while len(data) < n:
        packet = socket.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data


class Server(object):
    backlog = 1
    client = None

    def __init__(self, host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Error for using port
        self.socket.bind((host, port))
        self.socket.listen(self.backlog)

    def __del__(self):
        self.close()
        time.sleep(1)

    def accept(self):
        if self.client:
            self.client.close()
        
        self.client, self.client_addr = self.socket.accept()
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
    socket = None

    def __del__(self):
        self.close()
        time.sleep(1)

    def connect(self, host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((host, port))
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
            self.socket = None

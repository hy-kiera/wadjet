from jsonsocket import Server

BIND_IP = "127.0.0.1"
BIND_PORT = 9999

socket = Server(BIND_IP, BIND_PORT)

if __name__ =='__main__':
    socket.accept()
    
    while True:
        recv_data = socket.recv()
        if not recv_data:
            break        
        
        cmd = recv_data['cmd']

        send_data = {}
        if cmd =='q': # quit
            break
        else:
            send_data['data'] = 'unknown command ('+ cmd +')'

        socket.send(send_data)

    socket.close()
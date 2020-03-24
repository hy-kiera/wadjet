#!/usr/bin/env python
import os
import subprocess
import rospy
from jsonsocket import Client

HOST_IP = '127.0.0.1'
HOST_PORT = 9999

if __name__ == '__main__':
    rospy.init_node('codrone_node', anonymous=True)
    rate = rospy.Rate(2)

    filename = os.path.dirname(os.path.abspath(__file__)) + "/driver.py"

    subprocess.Popen(["python3", filename])
    rate.sleep()
    
    client = Client()
    client.connect(HOST_IP, HOST_PORT)

    while True:
        print("**********************************")
        print("command list : ")
        print("- q (quit)")
        cmd = raw_input("input command : ")

        send_data = {}
        send_data['cmd'] = cmd

        client.send(send_data)
        
        if (cmd == 'q' or cmd == 'Q'):
            print('quit')
            client.close()
            break

        print('recv : ')
        recv_data = client.recv()
        print(recv_data)

        rate.sleep()
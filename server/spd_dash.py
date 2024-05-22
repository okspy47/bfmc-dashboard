# Copyright (c) 2024, OkSpy (https://okspy.tech)
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notices, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notices,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the names of the copyright holders nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
from std_msgs.msg import Float32
import socket
import threading


server_ip = "172.20.10.6" # Connection data for Speedometer. Replace with your Raspberry Pi/Jetson IP.
server_port = 2004
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((server_ip, server_port))
server_socket.listen(5)


def spd_callback(data):
    """
    Callback function that reads the data from the specified ROS topic,
    then sends it to the Dashboard via the socket declared above.
    """
    string_data = str(data.data)
    client_sockets = list(client_connections.keys())
    for client_socket in client_sockets:
        try:
            encoded_data = string_data.encode('utf-8')
            client_socket.sendall(encoded_data)
        except Exception as ex:
            print("Error sending data to client:", ex)
            client_socket.close()
            if client_socket in client_connections:
                del client_connections[client_socket]

# Initialize ROS node and subscribe to the specified ROS topic.
rospy.init_node('ros_data2')
rospy.Subscriber('/speedspy', Float32, spd_callback)# Replace with your actual ROS Topic, if needed.


client_connections = {}
def handle_client(client_socket):
    """
    Function that handles the client connections, and if any client is connected,
    creates a separate thread with the Speedometer Data sending function.
    """
    try:
        while True:
            pass
    except Exception as ex:
        print("Error:", ex)
    finally:
        client_socket.close()
try:
    while True:
        client_socket, _ = server_socket.accept()
        print("Dashboard succesfully connected.")
        client_connections[client_socket] = True
        client_thread = threading.Thread(target=handle_client, args=(client_socket,))
        client_thread.start()
except Exception as ex:
    print("Error:", ex)
finally:
    server_socket.close()

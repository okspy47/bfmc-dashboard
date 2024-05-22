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
from std_msgs.msg import String
import socket

def sign_callback(data, client_socket):
    """
    This callback sends the received data from the defined ROS topic, 
    and sends it over the defined socket.
    """
    sign_info = data.data
    client_socket.send(sign_info.encode())

def main():
    """
    This function initiates the server socket connection, initiates the ROS Node, then 
    subscribes to the defined ROS topic.
    """
    rospy.init_node('sign_publisher', anonymous=True)
    host = '172.20.10.6' # Connection data for Signs. Replace with your Raspberry Pi/Jetson IP.
    port = 2001
    socket_adress=(host,port)
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print("Socket opened, listening at:", socket_adress)
    client_socket, address = server_socket.accept()
    print(f"Received connection from: {address}")

    rospy.Subscriber("detected_class", String, sign_callback, client_socket) # ROS subscriber to receive signs detected from the topic (change it according your needs).
    rospy.spin()

if __name__ == '__main__':
    main()

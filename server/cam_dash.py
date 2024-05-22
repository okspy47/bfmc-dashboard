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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import pickle
import struct
import imutils
import cv2

# Initialize ROS node
rospy.init_node('video_streamer')
bridge = CvBridge()

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_ip = "172.20.10.6" # Connection data for Video Feed. Replace with your Raspberry Pi/Jetson IP.
port = 2002
socket_address = (host_ip, port)
server_socket.bind(socket_address)
server_socket.listen(5)
print("Socket opened, listening at:", socket_address)

try:
    client_socket, addr = server_socket.accept()
    print('Received connection from:', addr)

    def image_callback(msg):
        try:
            """
            This function converts ROS Image Message (Matrix) to OpenCV video feed in 
            BGR8 format, then sends it to the Dashboard via the socket declared above.
            """
            frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
            frame = imutils.resize(frame, width=720)
            data = pickle.dumps(frame, 0)
            size = len(data)
            client_socket.send(struct.pack("L", size))
            client_socket.sendall(data)
        except Exception as e:
            print("Error processing image:", e)
    rospy.Subscriber("/camera/image_raw", Image, image_callback)  # ROS subscriber to receive images from the topic (change it according your needs).
    rospy.spin()

except KeyboardInterrupt:
    print("KeyboardInterrupt: Shutting down the server.")

finally:

    server_socket.close()
    cv2.destroyAllWindows()


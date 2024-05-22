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
import threading


current_gps_data = "0,0"

def update_gps_data():
    """
    Function that receives dummy GPS data inputed by the user,
    and stores it in the "current_gps_data" global variable.
    """
    global current_gps_data
    while not rospy.is_shutdown():
        print("Enter GPS Dummy Data:")
        new_data = input()
        if new_data:
            current_gps_data = new_data

def publish_test_data():
    """
    Function that initialises the ROS node and topic,
    then publishes the entered GPS Dummy data on the
    ROS topic. This is created for testing purposes.
    """
    rospy.init_node('test_gps_publisher', anonymous=True)
    pub = rospy.Publisher('/gps_data', String, queue_size=10) # Replace with your actual ROS Topic, if needed.
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        msg = String()
        msg.data = current_gps_data
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        update_thread = threading.Thread(target=update_gps_data)
        update_thread.start()
        publish_test_data()
    except rospy.ROSInterruptException:
        pass
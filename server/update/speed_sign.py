import rospy
from std_msgs.msg import Float32, String
import socket
import threading
import signal

# Server details
server_ip = "10.69.131.161"
server_port = 2006

# Create and set up the server socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((server_ip, server_port))
server_socket.listen(5)

# Dictionary to keep track of client connections
client_connections = {}

# Define a signal handler
def signal_handler(sig, frame):
    print('Shutting down server...')
    for client_socket in list(client_connections.keys()):
        client_socket.close()
        del client_connections[client_socket]
    server_socket.close()
    rospy.signal_shutdown('Server Shutdown')
    exit(0)

# Handle speed data callback
def spd_callback(data, client_socket):
    try:
        # Encode float data to string and send to client
        encoded_data = str(data.data).encode('utf-8')
        client_socket.sendall(encoded_data)
    except Exception as ex:
        print("Error sending speed data:", ex)
        client_socket.close()
        if client_socket in client_connections:
            del client_connections[client_socket]

# Handle sign data callback
def sign_callback(data, client_socket):
    try:
        # Send the sign info directly as string
        client_socket.send(data.data.encode())
    except Exception as ex:
        print("Error sending sign data:", ex)
        client_socket.close()
        if client_socket in client_connections:
            del client_connections[client_socket]

# Thread function to handle client connections
def handle_client(client_socket):
    rospy.Subscriber('/speedash', Float32, spd_callback, callback_args=client_socket)
    rospy.Subscriber('detected_class', String, sign_callback, callback_args=client_socket)
    try:
        while True:
            pass  # Maintain the connection
    except Exception as ex:
        print("Error with client:", ex)
    finally:
        client_socket.close()

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)

# Initialize ROS node
rospy.init_node('combined_node')

try:
    while True:
        client_socket, _ = server_socket.accept()
        print("Client connected.")
        client_connections[client_socket] = True
        client_thread = threading.Thread(target=handle_client, args=(client_socket,))
        client_thread.daemon = True  # Make thread daemon
        client_thread.start()
except Exception as ex:
    print("Error on server:", ex)
finally:
    server_socket.close()

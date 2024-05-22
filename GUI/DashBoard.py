# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
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


import socket
import pygame
from GUI.State import State
from objects.Map import Map
from objects.Button import Button
from objects.Button_Text import Button_Text
import time
from math import pi, sqrt, sin, cos
from objects.Object import Object
import json
import numpy as np
import struct
import pickle
import threading
import json
###########################################################
###########################################################
######################    Video    ########################
######################    Class    ########################
###########################################################
###########################################################
class Camera(Object):
    """
    Initialize a graphical logo button.

    Args:
        x (int): The x-coordinate of the button.
        y (int): The y-coordinate of the button.
        game: The game object.
        window: The window object.
        width (int, optional): The width of the button (default is 220).
        height (int, optional): The height of the button (default is 120).

    """

    def __init__(self, x, y, game, window, width=650, height=300):
        super().__init__(300, 0, game, window, width, height)
        image1 = self.game.image.load("setup/images/nosignal.png")
        
        self.frame = self.game.transform.scale(image1, (self.width, self.height))
        self.font = self.game.font.Font(None, 25)
        self.rectangle = self.game.Rect(x, y, self.width, self.height)
        self.on = False

    def change_frame(self, newFrame):
        """
        Change the displayed frame of the graphical logo button.

        Args:
            newFrame: The new frame data to be displayed.
        """
        try:
            if newFrame is not None and isinstance(newFrame, np.ndarray):
                if newFrame.ndim >= 2:
                    newFrame = np.rot90(newFrame)
                    newFrame = np.flip(newFrame, axis=0)
                    newSurface = self.game.surfarray.make_surface(newFrame)
                    self.frame = self.game.transform.scale(newSurface, (self.width, self.height))
                else:
                    print("Array does not have enough dimensions for rotation.")
            else:
                print("Invalid frame data provided.")
        except Exception as e:
            print(f"An error occurred: {e}")

    def draw(self):
        """
        Draw the graphical logo button on the surface.

        This method fills the surface with a background color and displays the current frame.

        """
        self.surface.fill(255)
        self.surface.blit(self.frame, (0, 0))

    def conn_lost(self):
        """
        Set the frame to the "No Signal" image when the connection is lost.

        """
        image1 = self.game.image.load("objects/images/nosignal.png")
        self.frame = self.game.transform.scale(image1, (self.width, self.height))

    def update(self):
        super().update()

###########################################################
###########################################################
######################    Signs    ########################
######################    Class    ########################
###########################################################
###########################################################

class Alerts(Object):
    def __init__(self, x, y, game, window, main_surface, size=600, sign_scale=0.4):

        self.names = {} 
        self.values = {}
        self.square = size / 5
        height = self.square * 5
        self.seconds_fadeaway = 3.0
        self.sign_scale = sign_scale
        self.lights = {}
        super().__init__(x, y, game, window, size, height)
        self.read()
        for name in self.names.keys():
            image = self.game.image.load("setup/images/signs/" + name + ".png") # Path to the signs images, in PNG format, with the background removed.
            image = self.game.transform.scale(image, (int(self.square * self.sign_scale), int(self.square * self.sign_scale)))
            self.lights[name] = image
        
        self.sgn_server_host = '172.20.10.6'  # Connection data for Traffic Signs. Replace with your Raspberry Pi/Jetson IP.
        self.sgn_server_port = 2001
        self.sgn_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sgn_client_socket.connect((self.sgn_server_host, self.sgn_server_port))
        self.detect_thread = threading.Thread(target=self.receive_sgn_data)
        self.detect_thread.start()
    
    def receive_sgn_data(self):
        """
        Receive the signs data read from the socket, and passes them to the drawing function.
        """
        while True:
            try:
                data = self.sgn_client_socket.recv(1024).decode()

                self.values[data] = "True"
            except Exception as e:
                print("Error receiving sign information:", e)

    def update(self, timePassed):
        """
        Update timers associated with named actions and set values to "False" when timers expire.

        Args:
            timePassed (float): The time passed, in seconds.

        """
        for key, value in self.names.items():
            self.names[key] = value - timePassed
            if self.names[key] < 0:
                self.values[key] = "False"

    def setValues(self, value):
        """
        Set a named value to "True" and initialize its associated timer.

        Args:
            value (str): The name of the value to be set.

        """
        self.values[value] = "True"
        self.names[value] = self.seconds_fadeaway

    def fill_with_image(self, image_surface):
        """
        Temporary workaround so we can combine the Main Surface's frame background with the signs frame background.
        """
        imagea = "setup/images/fill.jpeg"
        image_surface = pygame.image.load(imagea).convert_alpha()
        surface_width, surface_height = self.surface.get_size()
        for x in range(0, surface_width, image_surface.get_width()):
            for y in range(0, surface_height, image_surface.get_height()):
                self.surface.blit(image_surface, (x, y))

    def draw(self):
        """
        Draw the signs flagged as "True" on the surface without overlapping.

        """
        imagea = "setup/images/fill.jpeg"
        image_surface = pygame.image.load(imagea).convert_alpha()
        self.surface.fill(255)
        self.fill_with_image(image_surface)
        drawn_positions = set()
        for name in self.names:
            if self.values[name] == "True":
                position = self.calculate_position(name, drawn_positions)
                if position is not None:
                    self.lights[name].set_alpha(255)
                    self.surface.blit(self.lights[name], position)
                    drawn_positions.add(position)

    def calculate_position(self, name, drawn_positions):
        """
        Calculate position for the sign to avoid overlap, if their position is not predefined in the list below.

        Args:
            name (str): The name of the sign.
            drawn_positions (set): Set of positions already drawn.

        Returns:
            tuple or None: Position of the sign or None if it cannot be placed without overlap.
        """
        predefined_positions = { 
            "Car": (0, 450),
            "Pedestrian": (50, 450),
            "Crosswalksign": (100, 450),
            "Highwayentrancesign": (150, 450),
            "Highwayexitsign": (150, 450),
            "Parkingsign": (200, 450),
            "Onewayroadsign": (250, 450),
            "Stopsign": (300, 450),
            "No-entryroadsign": (350, 450),
            "Round-aboutsign": (400, 450),
            "Prioritysign": (450, 450),
            "carParking": (500, 450),
            "pedOnCross": (550, 450),
        }

        if name in predefined_positions:
            position = predefined_positions[name]
            if position not in drawn_positions:
                return position
            else:
                return None
        x = 0
        y = 0
        while (x, y) in drawn_positions:
            if x + self.square * self.sign_scale > self.surface.get_width():
                x = 0
                y += self.square * self.sign_scale
            else:
                x += self.square * self.sign_scale
            if x + self.square * self.sign_scale > self.surface.get_width() or y + self.square * self.sign_scale > self.surface.get_height():    
                x = self.surface.get_width() + 10  
        return (x, y)


    def read(self):
        """
        Read data from a JSON file to initialize names and values.

        This method reads data from a JSON file located at "setup/Alerts.json" to initialize
        the `names` and `values` attributes.

        """
        with open("setup/Alerts.json", "r") as f:
            self.data = json.load(f)
        self.names = self.data["names"]
        self.values = self.data["values"]


###########################################################
###########################################################
######################    Dash    #########################
######################    Class    ########################
###########################################################
###########################################################
class DashBoard(State):
    """
    Initialize a new instance of the class with various attributes.

    Args:
        game: The game object.
        window: The window object.
        pipeRecv (multiprocessing.Pipe): The pipe for receiving data.
        pipeSend (multiprocessing.Pipe): The pipe for sending data.
        speed (int, optional): The initial speed (default is 0).
        battery (int, optional): The initial battery level (default is 100).
        lane_error (int, optional): The initial lane error (default is 0).
        steer (int, optional): The initial steering value (default is 0).
        gps_data_show (string): Initial GPS positions (default is 0,0).
        spd_data_show (string): Initial Speed value (default is 0).
    """
    
    speed=0
    car_x=440
    car_y=330
    angle_change = 2
    steer_change = error_change = battery_dx = 1
    clicked = False
    global gps_data_show
    gps_data_show = "0,0"
    global spd_data_show
    spd_data_show = "08"

    
    def __init__(
        self,
        game,
        window,
        pipeRecv,
        pipeSend,
        initial_speed=0,
        speed=0, 
        position=(0,0),
        battery=100,
        lane_error=0,
    ):
        
        self.car_x, self.car_y = window.get_width() // 2, window.get_height() // 2
        self.map_offset_x, self.map_offset_y = 0, 0
        self.background_image = pygame.image.load("setup/images/bg.jpeg")
        self.background_image = pygame.transform.scale(self.background_image, (window.get_width(), window.get_height()))
        steer=0
        self.imu_file="setup/files/imu_values.txt"
        super().__init__(game, window)
        self.pipeRecv = pipeRecv
        self.pipeSend = pipeSend
        self.battery = battery
        self.lane_error = lane_error
        self.list = [1, 20, 33, 55, 66]
        self.cursor_pivot = (24, 24)   # The point where the image will rotate around it.
        self.sem = True
        self.names = {"load": 0, "save": 0, "reset": 0}
        self.battery_color = (0, 255, 0)
        self.seconds_fadeaway = 3
        self.little_car = self.game.image.load("setup/images/little_car.png") # File path for the car PNG.
        self.little_car = self.game.transform.scale(self.little_car, (30, 45)) # Car Size on the Minimap.
        self.steer = steer
        self.arrow = self.game.image.load("setup/images/arrow.png")
        self.arrow = self.game.transform.scale(self.arrow, (22, 22))
        self.arrow_pivot = (sin(self.little_car.get_width()),self.little_car.get_height() / 2)
        self.font_big = self.game.font.Font("setup/files/Designer.otf", 70)
        self.font_small = self.game.font.SysFont(None, 30)
        self.font_little = self.game.font.SysFont(None, 25)  
        self.buttonAutonomEnable = True
        self.buttonSpeedEnable = True
        self.button = Button(
            1125, 370, self.pipeSend, self.game, self.main_surface, "Auto"
        )
        self.button2 = Button(
            980, 370, self.pipeSend, self.game, self.main_surface, "Race"
        )

        self.map = Map(20, 20, self.game, self.main_surface,car_x=2820, car_y=3190) #  Minimap frame position related to the Main Surface.
        self.alerts = Alerts(1000, 240, self.game, self.main_surface, 250)  # Original Traffic Signs frame position related to the Main Surface.
        self.camera = Camera(850, 350, self.game, self.main_surface)  # Original Camera frame position related to the Main Surface.
        # self.buttonSave = Button_Text(550, 215, self.game, self.main_surface, "Save")
        # self.buttonLoad = Button_Text(625, 215, self.game, self.main_surface, "Load")
        # self.buttonReset = Button_Text(700, 215, self.game, self.main_surface, "Reset")
        self.objects = [self.alerts, self.camera] # List of the objects imported earlier that will be blitted on the Dashboard.


        import threading
        self.v_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.v_host_ip = '172.20.10.6'  # Connection data for Video Feed. Replace with your Raspberry Pi/Jetson IP.
        self.v_port = 2002
        self.v_client_socket.connect((self.v_host_ip, self.v_port))
        self.video_thread = threading.Thread(target=self.receive_video)
        self.video_thread.start()


        self.gps_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.gps_host_ip = '172.20.10.6'  # Connection data for GPS. Replace with your Raspberry Pi/Jetson IP.
        self.gps_port = 2100
        self.gps_client_socket.connect((self.gps_host_ip, self.gps_port))
        self.gps_thread = threading.Thread(target=self.receive_gps_data)
        self.gps_thread.start()
        gps_data_show = "0,0"
        
    
        self.spd_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.spd_host_ip = '172.20.10.6'  # Connection data for Speed. Replace with your Raspberry Pi/Jetson IP.
        self.spd_port = 2004
        self.spd_client_socket.connect((self.spd_host_ip, self.spd_port))
        self.spd_thread = threading.Thread(target=self.receive_spd_data)
        self.spd_thread.start()
        global spd_data_show
        spd_data_show= "25"


    def receive_video(self):
        """
        Function that receives and proceses the video frames received from the Video Feed socket declared above.
        """
        payload_size = struct.calcsize("Q")
        data = b""
        while True:
            while len(data) < payload_size:
                packet = self.v_client_socket.recv(4 * 1024)
                if not packet:
                    break
                data += packet
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            while len(data) < msg_size:
                data += self.v_client_socket.recv(4 * 1024)
            frame_data = data[:msg_size]
            data = data[msg_size:]
            frame = pickle.loads(frame_data)
            self.camera.change_frame(frame)
        

    def receive_gps_data(self):
        """
        Function that receives the GPS data received from the GPS socket declared above.
        """
        while True:
            try:
                data = self.gps_client_socket.recv(1024)
                if data:
                        global gps_data_show
                        gps_data_show = data.decode() 
                        self.update_map_position(gps_data_show)
            except Exception as e:
                print("Error receiving GPS data:", e)
                break


    def update_map_position(self, gps_data):
        """
        Update the map's position based on the received GPS data.

        Args:
            gps_data (str): The GPS data in the format "x,y".
        """
        try:
            x, y = map(float, gps_data.split(","))
            self.map_offset_x = x * 10 - self.car_x  # Adjust as needed, as my revamped map resolution is different from the original Bosch's Map SVG.
            self.map_offset_y = y * 10 - self.car_y  # Adjust as needed, as my revamped map resolution is different from the original Bosch's Map SVG.
            self.map.new_coordinates(self.map_offset_x, self.map_offset_y) # Move the map itself (not the car), based on the GPS data.
        except ValueError as e:
            print(f"Invalid GPS data: {gps_data}, Error: {e}")


    def receive_spd_data(self):
        """
        Function that receives the Speed data received from the Speed socket declared above.
        """
        while True:
            try:
                data_spd = self.spd_client_socket.recv(1024)
                if data_spd:
                        global spd_data_show
                        spd_data_show = data_spd.decode()
                        print(int(spd_data_show))
                    
            except Exception as e:
                print("Error receiving Speed data:", e)
                break

    def blitRotate(self, surf, image, pos, originPos, angle):
        """
        Rotate an image and blit it onto a surface.

        Args:
            surf: The target surface where the rotated image will be blitted.
            image: The image to be rotated and blitted.
            pos (tuple): The position (x, y) where the rotated image will be blitted.
            originPos (tuple): The pivot point (x, y) around which the image will be rotated.
            angle (float): The angle in degrees by which the image will be rotated.
        """
        image_rect = image.get_rect(
            topleft=(pos[0] - originPos[0], pos[1] - originPos[1])
        )
        offset_center_to_pivot = self.game.math.Vector2(pos) - image_rect.center
        rotated_offset = offset_center_to_pivot.rotate(-angle)
        rotated_image_center = (pos[0] - rotated_offset.x, pos[1] - rotated_offset.y)
        rotated_image = self.game.transform.rotate(image, angle)
        rotated_image_rect = rotated_image.get_rect(center=rotated_image_center)
        surf.blit(rotated_image, rotated_image_rect)

    def continous_update(self):
        """
        Continuously update the class attributes based on received messages.

        This method listens for incoming messages on the `pipeRecv` pipe and updates
        the class attributes accordingly, depending on the message type.
        """
        if self.pipeRecv.poll():
            msg = self.pipeRecv.recv()
            if msg["action"] == "steering":
                self.steer = msg["value"]
            elif msg["action"] == "modImg":
                self.camera.change_frame(msg["value"])
            elif msg["action"] == "map":
                terms_list = msg["value"].split()
                x = float(terms_list[1][0 : len(terms_list[1]) - 1]) * 150
                y = float(terms_list[3][0 : len(terms_list[3]) - 1]) * 150
                self.map.new_coordinates(x, y)
                self.map.update()
            elif msg["action"] == "battery":
                self.battery = msg["value"]
            elif msg["action"] == "roadOffset":
                self.little_car = self.game.transform.scale(
                    self.little_car, (330 + msg["value"], 430)
                )
            elif msg["action"] == "emptyAll":
                self.camera.conn_lost()

    def updateTimers(self, timePassed):
        """
        Update timers associated with named actions.

        This method updates timers for named actions stored in the `names` dictionary.
        It subtracts the specified `timePassed` from the timers.

        Args:
            timePassed (float): The time passed, in seconds.
        """
        for key, value in self.names.items():
            self.names[key] = value - timePassed

    def set_text(self, text):
        """
        Set a timer for a named action.

        This method sets a timer for a named action specified by the `text` parameter.
        The timer is initially set to 3.0 seconds.

        Args:
            text (str): The name of the action.
        """
        self.names[text] = 3.0

    def update(self):
        """
        Update the class state.

        This method updates the class state by performing the following actions:
        1. Calls the superclass's update method using `super()`.
        2. Calls the `continous_update` method to process incoming messages and update attributes.
        3. Calls the `input` method to handle user input.
        4. Adjusts the `battery_color` attribute based on the current battery level.
        """
        super().update()
        self.continous_update()
        with open(self.imu_file, "r") as f: 
            try:
                time.sleep(0.2)
                values = f.read().split()
                if len(values) >= 1:
                    self.steer = float(values[0])
                else:
                    print("Not enough values in the file.")

            except ValueError:
                print("Error reading the data from file. Using 0 as default value.")
                self.steer = 0

        self.input()
        self.battery_color = (
            (100 - self.battery) * 255 / 100,
            (self.battery / 100) * 255,
            0,
        )


    def rad_to_degrees(self, angle):
        """
        Convert an angle from radians to degrees.

        Args:
            angle (float): The angle in radians to be converted.

        """
        converted = angle * 180 / pi
        return converted

    def deg_to_radians(self, angle):
        """
        Convert an angle from degrees to radians.

        Args:
            angle (float): The angle in degrees to be converted.

        """
        converted = angle * pi / 180
        return converted

    def draw(self):
        """
        Draw the graphical elements on the main surface.

        This method clears the main surface, draws operation success messages with fading,
        draws various objects, buttons, battery level, speed image, team and BFMC's logos and the little car image.

        """
        self.main_surface.fill(0)
        self.main_surface.blit(self.background_image, (0, 0))

        for object in self.objects:
            object.draw()
        self.alerts.draw()

        self.main_surface.blit(self.alerts.surface, (330,0))  # Traffic Signs Surface Position 
        self.main_surface.blit(self.camera.surface, (310, 0)) # Camera Surface Position
        
        if self.buttonAutonomEnable: # Race Button Draw function
            self.button.draw()
        if self.buttonSpeedEnable: # Auto Button Draw function
            self.button2.draw()
       
        battery_show = self.font_small.render(
            str(self.battery) + "%", True, self.battery_color
        )
        self.main_surface.blit(battery_show, (610, 420))   # Battery Frame Position related to the Main Surface
        
        self.game.draw.line(self.main_surface, (255, 255, 255), (455, 420), (515, 300), 5) # Left Lane Assist line drawing.
        self.game.draw.line(self.main_surface, (255, 255, 255), (750, 300), (810, 420), 5) # Right Lane Assist line drawing.

        self.map.draw()
        rotated_car = self.game.transform.rotate(self.little_car, -self.steer) # The car on the minimap drawing.
        car_rect = rotated_car.get_rect(center=(145, 80))
        self.main_surface.blit(rotated_car, car_rect)


        self.game.draw.arc( # Battery Arc drawing function.
            self.main_surface,
            self.battery_color,
            [435, 390, 400, 100],           
            pi / 4 + (100 - self.battery) * (pi / 2) / 100,
            pi - pi / 4,
            25,

        )
        pygame.transform.rotate(
            self.main_surface,90
        )
        
        arrow_x = 155
        arrow_y = 50

        self.blitRotate(
            self.main_surface, self.arrow, (arrow_x, arrow_y), self.arrow_pivot, 90 + -self.steer # Car Rotating function on the Minimap, according to yaw values.
        )
        if self.steer > 0:
            steer_show = self.font_little.render(
                "+" + str(self.steer) + "°", True, (255, 255, 255) 
            )
            self.main_surface.blit(steer_show, (775, 310)) # Steer value (Yaw) blitting on the top of the Right Lane assist, if it's a positive value.
        elif self.steer <= 0:
            steer_show = self.font_little.render(
                str(self.steer) + "°", True, (255, 255, 255)
            )
            self.main_surface.blit(steer_show, (430, 310)) # Steer value (Yaw) blitting on the top of the Left Lane assist, if it's a negative value.

        speed_show = self.font_big.render(str(spd_data_show), True, (255, 255, 255)) # Speed blit on the Main Surface.
        self.main_surface.blit(speed_show, (583, 315))

        r_img = self.game.image.load("setup/images/rebooters.png") # Team Logo blitting on the Main Surface.
        logo_rebooters = pygame.transform.scale(r_img, (300, 205))
        self.main_surface.blit(logo_rebooters, (0, 370))

        b_img = self.game.image.load("setup/images/BFMC.png") # BFMC's Logo blitting on the Main Surface.
        logo_bfmc = pygame.transform.scale(b_img, (220, 120))
        self.main_surface.blit(logo_bfmc, (1050, 0))

        gps_data_blit = self.font_little.render(f"X,Y: {gps_data_show}", True, (255, 255, 255)) # X,Y GPS Data Blitting below the minimap
        self.main_surface.blit(gps_data_blit, (90, 220))
        
        super().draw()


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


from objects.Object import Object
import paramiko

class Button(Object):
    """
    Initialize a toggle button with images for "on" and "off" states. Also added the posibility to connect to your machine via SSH protocol.

    Args:
        x (int): The x-coordinate of the button.
        y (int): The y-coordinate of the button.
        pipe: The pipe for communication.
        game: The game object.
        window: The window object.
        text (str, optional): The text label for the button (default is empty).
        width (int, optional): The width of the button (default is 120).
        height (int, optional): The height of the button (default is 120).
        remote_host: The Raspberry Pi/Jetson IP
        username: The Raspberry Pi/Jetson Username
        password: The Raspberry Pi/Jetson Password

    """

    def __init__(self, x, y, pipe, game, window, text="", width=130, height=130, remote_host="remote_host_ip", username="username", password="password"):
        super().__init__(x, y, game, window, width, height)
        image1 = self.game.image.load("setup/images/stop.png") # Path for the Stop Engine button image.
        image1.set_colorkey((0, 0, 0))
        self.surface.convert(image1)
        image1 = self.game.transform.scale(image1, (self.width, self.height))
        image2 = self.game.image.load("setup/images/start.png") # Path for the Start Engine button image.
        image2 = self.game.transform.scale(image2, (self.width, self.height))
        self.pipe = pipe
        self.font = self.game.font.Font(None, 25)
        self.rectangle = self.game.Rect(x, y, self.width, self.height)
        self.states = {}
        self.text = text
        self.states["on"] = image1
        self.states["off"] = image2
        self.on = False
        self.remote_host = "172.20.10.6"  # Connection data for the Raspberry Pi/Jetson
        self.username = "misu"
        self.password = "misu"


    def execute_remote_script(self, script_path):
        """
        Execute a script on a remote machine via SSH.

        Args:
            script_path (str): Path to the script on the remote machine.
        """
        try:
            ssh_client = paramiko.SSHClient()
            ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh_client.connect(hostname=self.remote_host, username=self.username, password=self.password)
            command = f'{script_path}'
            stdin, stdout, stderr = ssh_client.exec_command(command)
            for line in stdout: # For showing the output of the executed command on the remote terminal.
                print(line.strip())
            for line in stderr:
                print(line.strip())
            ssh_client.close()
        except Exception as e:
            print("Error executing remote script:", e)

    def colliding(self, mousePos):
        """
        Check if the mouse position collides with the button's rectangle.

        Args:
            mousePos (tuple): The mouse position as a tuple (x, y).

        Returns:
            bool: True if the mouse position collides with the button's rectangle, False otherwise.
        """
        if self.rectangle.collidepoint(mousePos):
            return True
        else:
            return False

    def draw(self):
        """
        Draw the toggle button on the surface.

        This method fills the surface with a background color and displays the appropriate image
        based on the current state ("on" or "off"). It also renders the button's text label.

        """
        self.surface.fill(0)
        if self.on:
            
            self.surface.convert(self.states["on"])
            self.surface.set_colorkey((0, 0, 0))
            self.surface.blit(self.states["on"], (0, 0))
        else:
            self.surface.convert(self.states["off"])
            self.surface.set_colorkey((0, 0, 0))
            self.surface.blit(self.states["off"], (0, 0))
        text_x = self.width // 3
        text_y = 3 * self.height // 5.2
        text_surface = self.font.render(self.text, True, (255, 255, 255))
        self.surface.blit(text_surface, (text_x, text_y))
        super().draw()

    def update(self):
        """
        Update the toggle button's state and send control commands.

        This method updates the state of the toggle button and sends control commands based on
        whether the button is in the "on" or "off" state. It sends commands to start or stop
        the engine, reset steering, and set speed to zero.

        Also, using the execute_remote_script function, you can define and execute a remote
        script on the Raspberry Pi/Jetson, like ROS Launch files, Python Scripts, etc.

        """
        super().update()
        if self.on is False:
            self.execute_remote_script("Desktop/hello.py") # Example of using the Remote Script Execution function.
            self.on = True
        else:
            self.pipe.send({"action": "startEngine", "value": False})
            self.pipe.send({"action": "steer", "value": 0}) # Example of using pipes to communicate with the car's steering and engine.
            self.pipe.send({"action": "speed", "value": 0})
            self.on = False

# Reworked BFMC Dashboard

## Project Overview
Welcome! This project is an enhanced version of the original BFMC Dashboard found on their [GitHub](https://github.com/ECC-BFMC/Computer/tree/main/Dashboard). It has been redesigned to offer improved usability, additional features, and a more sleek design. It was also featured on the semifinals of the BFMC 2024 Edition. The project is now ROS Based, only the buttons can be configured to send serial commands via the [Brain](https://github.com/ECC-BFMC/Brain?tab=readme-ov-file).

## Key Features
- **Fully redesigned**: New UI, redesigned to look like a digital modern dashboard.
- **Socket Communication**: Different kind of data, like Video Feed, Speed, Signs and GPS data are sent between the Car and the Dashboard via Sockets.
- **ROS Implementation**: The server scripts(that runs on the car) reads data from ROS Topics and and sends them via Socket. Also, I included scripts that can write specific dummy data on the ROS topics for testing purposes.


## Before & After
### Before:
![BFMC Dashboard](https://bosch-future-mobility-challenge-documentation.readthedocs-hosted.com/_images/dashboard.png)
### After:
![Reworked BFMC Dashboard](https://i.imgur.com/XSp5NbL.png)

## Installation & Using the Dashboard
### Server Side (The Car):

Copy the files inside of the "server" folder on your Raspberry Pi/Jetson, and configure the IP & Ports for each script, according your needs.

```bash
# In Separate terminals (Update with all sockets in one file coming soon)

python3 cam_dash.py # Starts to read the Video ROS Topic and launchs the socket.
python3 gps_dash.py # Starts to read the GPS ROS Topic and launchs the socket.
python3 sgn_dash.py # Starts to read the Signs ROS Topic and launchs the socket.
python3 spd_dash.py # Starts to read the Speed ROS Topic and launchs the socket.
```
### Client Side (The Computer that will run the Dashboard):

Clone the repository and configure the IP & Ports for each socket, according your needs. All of them are located in `GUI/DashBoard.py`, except for an particular one(the SSH one), located in `objects/Button.py`
```bash
# Clone the repository.
git clone https://github.com/okspy47/bfmc-dashboard

# Navigate to the cloned project.
cd bfmc-dashboard

# Install required Python libraries.
pip3 install -r requirements.txt

# Run the Dashboard (Will only start if the Server Side sockets are launched and active).
python3 Dashboard.py
```

### Optional - For Testing
```bash
# This set of scripts are intended for testing the capabilities of the Dashboard even if
# you don't have a car. Basically, just inserts dummy data on the specified ROS Topics.

cd server/ROS

# In separate terminals:
python3 cam.py # Starts the Dummy Video Feed ROS Publishing - Will use your device video camera.
python3 gps.py # Starts the Dummy GPS Data ROS Publishing.
python3 sgn.py # Starts the Dummy Signs Data ROS Publishing.
python3 spd.py # Starts the Dummy Speed Data ROS Publishing.

```

## Future Updates
| Name  | Status  |
|:----------|:----------|
| Transferring IMU Data communication logic to ROS Topic.    |❌|
| Unifying and optimizing socket communication.    |❌|
| Unifying Dummy Data ROS publishing.    |❌|
| Mac OS compatibility (Working at the moment via Parallels).    |❌|
| Map redrawing for the next year track.    |❌|

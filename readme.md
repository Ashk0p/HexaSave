#  Hexapod Field Robotics Platform (ROS 2 Lyrical)

A fully autonomous, field-ready hexapod control architecture developed in Python for Ubuntu running **ROS 2 Lyrical Luth**. The platform features real-time target tracking via YOLO, a custom Inverse Kinematics (IK) gait controller tuned for high-friction hardware, micro-controller serial bridging over dual USB buses, and a responsive web-based HUD for remote field teleoperation.

---

##  Key Features

* **Headless Field HUD:** Flask-based Web Interface streaming live `mjpeg` camera feeds and publishing real-time joystick commands via `rosbridge_websocket` (Port `9090`).
* **Multi-Mode Operational Pipeline:**
  * **Teleop:** Direct touch-joystick velocity control via `nipplejs`.
  * **Auto Follow:** Autonomous tracking using YOLO vision models (`yolo11n.onnx`) and dynamic `Twist` target centering.
  * **Crab Rave Mode:** Dynamic direct-to-bridge joint kinematic demonstration.
* **Custom Inverse Kinematics & Gait Engine:**
  * Tripod gait system with middle-leg leverage locked coxa support for maximum stability.
  * Dedicated high-speed multithreaded Python `servo_bridge` interface to dual ESP microcontrollers (`/dev/ttyUSB0` & `/dev/ttyUSB1`).
* **Environment Stability:** Built natively to handle clean IPC without distro ABI collisions.

---

## 🏗 System Architecture

```
                                +-------------------+
                                |   Web HUD (Phone) |
                                +---------+---------+
                                          |
                                    (WebSocket)
                                          v
                              +-----------+-----------+
                              |   rosbridge_server    |
                              +-----------+-----------+
                                          |
                                    (/robot_mode)
                                          v
+------------------+         +------------+------------+         +------------------+
|   pi_camera      | ------> |    person_follower     | ------> |  gait_controller |
|  (rpicam-vid)    |         | (YOLO Tracking Filter)  |         |   (Tripod IK)    |
+------------------+         +-------------------------+         +--------+---------+
                                                                          |
                                                        (/hexapod_joint_controller/commands)
                                                                          v
                                                                 +--------+---------+
                                                                 |   servo_bridge   |
                                                                 +----+--------+----+
                                                                      |        |
                                                          (/dev/ttyUSB0)      (/dev/ttyUSB1)
                                                                      v        v
                                                                   [ESP 1]  [ESP 2]
```

---

## 🛠 Hardware & System Requirements

* **SBC:** Raspberry Pi 4 / 5 running Ubuntu.
* **OS Environment:** ROS 2 Lyrical Luth (LTS).
* **Actuation Drivers:** 2x ESP Microcontrollers (ESP8266/ESP32) connected via USB Serial at 115200 Baud.
* **Camera:** Raspberry Pi Camera Module.

---

## Installation & Setup

### 1. System Dependencies
Install ROS 2 dependencies and Python packages system-wide:

```bash
sudo apt update
sudo apt install -y \
  python3-flask \
  python3-serial \
  ros-lyrical-rosbridge-server \
  ros-lyrical-web-video-server
```

### 2. Permissions Setup
Grant your user permission to access hardware video streams, serial connections, and DMA buffers:

```bash
sudo usermod -aG video,render,dialout $USER
```
> **Note:** Re-login or restart your SSH session for group permissions to take effect.

### 3. Build Workspace
Clean and compile the ROS 2 workspace specifically under the Lyrical environment:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/hexapod_ws
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

---

## 📱 Field Operations & Quick Start

### Option 1: Full System Web Stack (Recommended)
Launch the Flask web server to control the robot from any phone connected to the Pi's Wi-Fi hotspot:

```bash
python3 ~/hexapod_ws/hexapod_web_app.py
```
1. Open your browser on your phone and navigate to `http://<PI_IP_ADDRESS>:5000`.
2. Tap **Boot Stack** to spin up ROS 2 nodes in the background.
3. Switch seamlessly between **Teleop** (Joystick) and **Auto Follow** modes.

---

### Option 2: Manual Terminal Launch
To manually run the perception and motion pipeline without the Web GUI:

```bash
source /opt/ros/lyrical/setup.bash
source ~/hexapod_ws/install/setup.bash
ros2 launch hexapod_perception hexapod_robot.launch.py
```

Optional launch arguments:
* `servos:=false` — Run in simulation mode without sending commands to physical ESP hardware.
* `record:=true` — Enable automatic review recording (~/hexapod_ws/video).

---


## 📡 Topic Reference

| Topic | Type | Source | Description |
| :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Web HUD / Follower | Velocity commands for linear & angular motion. |
| `/robot_mode` | `std_msgs/msg/String` | Web HUD | Mode selector (`teleop` vs `follow`). |
| `/hexapod_joint_controller/commands` | `std_msgs/msg/Float64MultiArray` | Gait Controller / Rave | 18-element radian vector for joint positions. |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Camera Node | Stream input for YOLO tracking & Web Video Server. |

---

## 📜 License
Distributed under the MIT License. See `LICENSE` for more information.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import threading
import math

class ServoBridge(Node):
    def __init__(self):
        super().__init__('servo_bridge')

        # Subscribe to the gait controller's output
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/hexapod_joint_controller/commands',
            self.joint_command_callback,
            10
        )

        # ---------------------------------------------------------
        # SERIAL CONFIGURATION
        # Update these ports to match your actual ESP8266 USB ports
        # ---------------------------------------------------------
        self.baud_rate = 115200  # HIGH SPEED to prevent serial lag
        try:
            self.serial1 = serial.Serial('/dev/ttyUSB0', self.baud_rate, timeout=0.01)
            self.get_logger().info("Connected to ESP 1 on /dev/ttyUSB0")
        except serial.SerialException:
            self.get_logger().error("Failed to connect to ESP 1 on /dev/ttyUSB0")
            self.serial1 = None

        try:
            self.serial2 = serial.Serial('/dev/ttyUSB1', self.baud_rate, timeout=0.01)
            self.get_logger().info("Connected to ESP 2 on /dev/ttyUSB1")
        except serial.SerialException:
            self.get_logger().error("Failed to connect to ESP 2 on /dev/ttyUSB1")
            self.serial2 = None

        # ---------------------------------------------------------
        # HARDWARE DIRECTION INVERSIONS
        # 1 = Normal, -1 = Inverted. Adjust these if a joint moves backward.
        # [Coxa, Femur, Tibia] for Legs 0 through 5
        # ---------------------------------------------------------
        self.directions = [
            -1, -1, -1,   # Leg 0 (Right Front)
            -1, -1, -1,   # Leg 1 (Right Middle) - Coxa is hard-locked in gait controller
            -1, -1, -1,   # Leg 2 (Right Rear)
             1,  1,  1,   # Leg 3 (Left Front)
             1,  1,  1,   # Leg 4 (Left Middle)
             1,  1,  1    # Leg 5 (Left Rear)
        ]

    def joint_command_callback(self, msg):
        if len(msg.data) != 18:
            return

        # Convert radians (-pi/2 to pi/2) to degrees (0 to 180, centered at 90)
        degrees = []
        for i in range(18):
            angle_rad = msg.data[i] * self.directions[i]
            angle_deg = int(math.degrees(angle_rad) + 90.0)
            
            # Clamp to safe servo limits
            angle_deg = max(0, min(180, angle_deg))
            degrees.append(angle_deg)

        # Split the 18 joints between the two ESP8266 boards (9 per board)
        # Assumes Right legs (0-8) are on ESP1, Left legs (9-17) are on ESP2
        payload1_str = ",".join(map(str, degrees[0:9])) + "\n"
        payload2_str = ",".join(map(str, degrees[9:18])) + "\n"

        payload1 = payload1_str.encode('utf-8')
        payload2 = payload2_str.encode('utf-8')

        # ---------------------------------------------------------
        # THREADED SERIAL WRITES (Fixes the sync delay)
        # ---------------------------------------------------------
        def write_esp1():
            if self.serial1 and self.serial1.is_open:
                self.serial1.write(payload1)
                self.serial1.flush()

        def write_esp2():
            if self.serial2 and self.serial2.is_open:
                self.serial2.write(payload2)
                self.serial2.flush()

        # Fire both writes simultaneously
        t1 = threading.Thread(target=write_esp1)
        t2 = threading.Thread(target=write_esp2)

        t1.start()
        t2.start()

        # Wait for both to finish to keep the callback clean
        t1.join()
        t2.join()

def main(args=None):
    rclpy.init(args=args)
    node = ServoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial1:
            node.serial1.close()
        if node.serial2:
            node.serial2.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

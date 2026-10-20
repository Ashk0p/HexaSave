import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class GaitController(Node):
    def __init__(self):
        super().__init__('gait_controller')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/hexapod_joint_controller/commands',
            10
        )
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Tripod Gait parameters
        self.timer_period = 0.02
        self.step_freq = 0.8       
        self.step_height = 0.05    
        self.step_length = 0.35    # High stride for the 4 corner propulsion legs
        
        # Stance parameters tuned for steel chassis
        self.default_x = 0.16      
        self.default_y = 0.00
        self.default_z = -0.11     

        self.coxa_len = 0.05
        self.femur_len = 0.10
        self.tibia_len = 0.15

        self.cmd_linear_x = 0.0
        self.cmd_linear_y = 0.0
        self.cmd_angular_z = 0.0

        self.hip_yaws = [-0.7854, -1.5708, -2.3562, 0.7854, 1.5708, 2.3562]
        self.hip_x = [0.12, 0.0, -0.12, 0.12, 0.0, -0.12]
        self.hip_y = [-0.10, -0.15, -0.10, 0.10, 0.15, 0.10]

        # Standard Tripod Groupings
        self.group_a_legs = [0, 2, 4]  # Right Front, Right Rear, Left Middle
        self.group_b_legs = [3, 5, 1]  # Left Front, Left Rear, Right Middle

        self.phase = 0.0
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def cmd_vel_callback(self, msg):
        self.cmd_linear_x = max(-0.8, min(0.8, msg.linear.x))
        self.cmd_linear_y = max(-0.8, min(0.8, msg.linear.y))
        self.cmd_angular_z = max(-1.0, min(1.0, msg.angular.z))

    def solve_ik(self, x, y, z):
        coxa = math.atan2(y, x)
        r = math.hypot(x, y) - self.coxa_len
        d = max(abs(self.femur_len - self.tibia_len) + 0.001, 
                min(self.femur_len + self.tibia_len - 0.001, math.hypot(r, z)))

        cos_femur2 = max(-1.0, min(1.0, (self.femur_len**2 + d**2 - self.tibia_len**2) / (2 * self.femur_len * d)))
        cos_tibia = max(-1.0, min(1.0, (self.femur_len**2 + self.tibia_len**2 - d**2) / (2 * self.femur_len * self.tibia_len)))

        femur = math.atan2(z, r) + math.acos(cos_femur2)
        tibia = -(math.pi - math.acos(cos_tibia)) + (math.pi / 2.0) # Hardware Tibia patch
        return coxa, femur, tibia

    def control_loop(self):
        is_moving = abs(self.cmd_linear_x) > 0.01 or abs(self.cmd_linear_y) > 0.01 or abs(self.cmd_angular_z) > 0.01
        
        if is_moving:
            self.phase = (self.phase + self.step_freq * self.timer_period) % 1.0
        else:
            if 0.0 < self.phase < 1.0:
                self.phase += self.step_freq * self.timer_period
                if self.phase >= 1.0:
                    self.phase = 0.0

        joint_commands = [0.0] * 18

        for leg_idx in range(6):
            is_group_a = leg_idx in self.group_a_legs
            leg_phase = self.phase if is_group_a else (self.phase + 0.5) % 1.0

            yaw = self.hip_yaws[leg_idx]
            
            fx_global = self.hip_x[leg_idx] + (self.default_x * math.cos(yaw))
            fy_global = self.hip_y[leg_idx] + (self.default_x * math.sin(yaw))

            vx_global = self.cmd_linear_x - (fy_global * self.cmd_angular_z)
            vy_global = self.cmd_linear_y + (fx_global * self.cmd_angular_z)

            # Calculate normal stride for the leg
            stride_x = (vx_global * math.cos(-yaw) - vy_global * math.sin(-yaw)) * self.step_length
            stride_y = (vx_global * math.sin(-yaw) + vy_global * math.cos(-yaw)) * self.step_length

            # THE FIX: If it is a middle leg (1 or 4), zero out its horizontal stride.
            # It will still lift (z-axis) in phase with its tripod group for leverage, but won't sweep.
            if leg_idx in [1, 4]:
                stride_x = 0.0
                stride_y = 0.0

            if not is_moving and self.phase == 0.0:
                x, y, z = self.default_x, self.default_y, self.default_z
            elif leg_phase < 0.5:
                # Swing Phase (Leg lifts up and steps forward)
                s_phase = leg_phase / 0.5
                x = self.default_x - (stride_x / 2.0) + (s_phase * stride_x)
                y = self.default_y - (stride_y / 2.0) + (s_phase * stride_y)
                z = self.default_z + (self.step_height * math.sin(math.pi * s_phase))
            else:
                # Stance Phase (Leg presses down and pushes backward)
                s_phase = (leg_phase - 0.5) / 0.5
                x = self.default_x + (stride_x / 2.0) - (s_phase * stride_x)
                y = self.default_y + (stride_y / 2.0) - (s_phase * stride_y)
                z = self.default_z

            coxa, femur, tibia = self.solve_ik(x, y, z)

            # Ensure middle leg Coxas never actuate, acting as passive leverage stilts
            if leg_idx in [1, 4]:
                coxa = 0.0

            base_idx = leg_idx * 3
            joint_commands[base_idx] = coxa
            joint_commands[base_idx + 1] = femur
            joint_commands[base_idx + 2] = tibia

        msg = Float64MultiArray()
        msg.data = joint_commands
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GaitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

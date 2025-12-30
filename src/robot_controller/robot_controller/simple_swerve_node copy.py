import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class DiffSwerveModule:
    def __init__(self, name, x_offset, y_offset, steer_kp=2.0, max_steer_speed=5.0):
        """
        Initialize a single differential swerve module.
        
        :param name: Name of the module (e.g. 'FL')
        :param x_offset: X position relative to robot center (meters)
        :param y_offset: Y position relative to robot center (meters)
        :param steer_kp: Proportional gain for steering control.
        :param max_steer_speed: Maximum steering simulation speed (rad/s) for dummy feedback.
        """
        self.name = name
        self.x = x_offset
        self.y = y_offset
        self.kp = steer_kp
        self.max_steer_speed = max_steer_speed
        
        self.current_angle = 0.0 # Feedback State
        self.target_angle = 0.0
        self.target_speed = 0.0

    def update(self, vx, vy, omega, dt):
        """
        Update module state and calculate motor outputs.
        
        :param vx, vy, omega: Robot velocity command
        :param dt: Time step in seconds
        :return: (v_left, v_right)
        """
        # 1. Kinematics (Instantaneous Target)
        vx_wheel = vx - (omega * self.y)
        vy_wheel = vy + (omega * self.x)
        
        speed = math.sqrt(vx_wheel**2 + vy_wheel**2)
        angle = math.atan2(vy_wheel, vx_wheel)
        
        # 2. Optimization
        speed, angle = self._optimize_steering(speed, angle, self.current_angle)
        
        self.target_speed = speed
        self.target_angle = angle
        
        # 3. Dummy Physics Simulation (Simulate Sensor Lag)
        # Move current_angle towards target_angle with max speed limit
        angle_diff = self.target_angle - self.current_angle
        # Normalize diff
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        
        step = self.max_steer_speed * dt
        if abs(angle_diff) < step:
            self.current_angle = self.target_angle
        else:
            self.current_angle += math.copysign(step, angle_diff)
            
        # Normalize current_angle for cleanliness
        self.current_angle = (self.current_angle + math.pi) % (2 * math.pi) - math.pi

        # 4. Control Logic (Differential Mixing)
        # Re-calculate error based on the new simulated current_angle
        error = self.target_angle - self.current_angle
        error = (error + math.pi) % (2 * math.pi) - math.pi
        
        steer_effort = error * self.kp
        
        # Drive: Left = -Speed, Right = +Speed (Standard differential logic validation needed, typically opposites)
        # Steer: Left = +Effort, Right = +Effort (Both turn in same direction)
        
        v_left = -self.target_speed + steer_effort
        v_right = self.target_speed + steer_effort
        
        return v_left, v_right

    def _optimize_steering(self, target_speed, target_angle, current_angle):
        """
        Optimize steering to avoid turning more than 90 degrees.
        """
        
        # Normalize difference to [-pi, pi]
        diff = target_angle - current_angle
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        
        # If we need to turn more than 90 degrees, flip the wheel direction and reverse speed
        if abs(diff) > math.pi / 2:
            target_angle = target_angle + math.pi # Flip direction
            target_angle = (target_angle + math.pi) % (2 * math.pi) - math.pi
            target_speed = -target_speed
            
        return target_speed, target_angle

class SimpleSwerveNode(Node):
    def __init__(self):
        super().__init__('simple_swerve_controller')
        
        self.declare_parameter('wheel_base', 0.5)
        self.declare_parameter('track_width', 0.5)
        
        wb = self.get_parameter('wheel_base').value
        tw = self.get_parameter('track_width').value
        
        lx = wb / 2.0
        ly = tw / 2.0
        
        self.modules = [
            DiffSwerveModule("FL", lx, ly),
            DiffSwerveModule("FR", lx, -ly),
            DiffSwerveModule("BL", -lx, ly),
            DiffSwerveModule("BR", -lx, -ly)
        ]
        
        self.input_cmd = [0.0, 0.0, 0.0] # vx, vy, omega
        
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.vis_pub = self.create_publisher(Float64MultiArray, '/swerve_state', 10)
        self.create_timer(0.05, self.control_loop) # 20Hz (Slower for vis/logging)
        
        self.get_logger().info("Simple Diff Swerve Controller (Feedback Sim) Started")

    def cmd_vel_cb(self, msg):
        self.input_cmd = [msg.linear.x, msg.linear.y, msg.angular.z]

    def control_loop(self):
        vx, vy, omega = self.input_cmd
        dt = 0.05
        
        state_array = []
        log_str = "Status: "
        for mod in self.modules:
            vl, vr = mod.update(vx, vy, omega, dt)
            # Log simpler format: Name: CurrAng -> TargAng | L/R Vel
            log_str += f"[{mod.name}: Cur={mod.current_angle:.2f}->Trg={mod.target_angle:.2f} | L={vl:.2f}, R={vr:.2f}] "
            
            # Append to state array: [Ang, VL, VR] per module
            state_array.extend([mod.current_angle, vl, vr])
            
        if self.input_cmd != [0.0, 0.0, 0.0] or any(abs(m.current_angle - m.target_angle) > 0.01 for m in self.modules):
             self.get_logger().info(log_str)
        
        # Publish State
        msg = Float64MultiArray()
        msg.data = state_array
        self.vis_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSwerveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


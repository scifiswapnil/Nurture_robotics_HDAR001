import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class VelocitySmoother(Node):
    def __init__(self):
        super().__init__('velocity_smoother')
        
        # Parameters
        self.declare_parameter('max_accel_linear', 0.5) # m/s^2
        self.declare_parameter('max_accel_angular', 1.0) # rad/s^2
        self.declare_parameter('frequency', 20.0) # Hz
        
        self.max_accel_linear = self.get_parameter('max_accel_linear').value
        self.max_accel_angular = self.get_parameter('max_accel_angular').value
        self.frequency = self.get_parameter('frequency').value
        
        # State
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        # Pub/Sub
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.pub_smooth = self.create_publisher(Twist, '/smooth_cmd_vel', 10)
        
        self.timer = self.create_timer(1.0/self.frequency, self.control_loop)
        
        self.get_logger().info(f"Velocity Smoother Started: LinAccel={self.max_accel_linear}, AngAccel={self.max_accel_angular}")

    def cmd_callback(self, msg):
        self.target_linear_x = msg.linear.x
        self.target_linear_y = msg.linear.y
        self.target_angular_z = msg.angular.z

    def control_loop(self):
        dt = 1.0 / self.frequency
        
        # Smooth Linear X
        diff_x = self.target_linear_x - self.current_linear_x
        step_x = self.max_accel_linear * dt
        if abs(diff_x) < step_x:
            self.current_linear_x = self.target_linear_x
        else:
            self.current_linear_x += math.copysign(step_x, diff_x)
            
        # Smooth Linear Y
        diff_y = self.target_linear_y - self.current_linear_y
        step_y = self.max_accel_linear * dt # Assuming same linear accel limit
        if abs(diff_y) < step_y:
            self.current_linear_y = self.target_linear_y
        else:
            self.current_linear_y += math.copysign(step_y, diff_y)

        # Smooth Angular Z
        diff_z = self.target_angular_z - self.current_angular_z
        step_z = self.max_accel_angular * dt
        if abs(diff_z) < step_z:
            self.current_angular_z = self.target_angular_z
        else:
            self.current_angular_z += math.copysign(step_z, diff_z)
            
        # Publish
        msg = Twist()
        msg.linear.x = self.current_linear_x
        msg.linear.y = self.current_linear_y
        msg.angular.z = self.current_angular_z
        self.pub_smooth.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocitySmoother()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

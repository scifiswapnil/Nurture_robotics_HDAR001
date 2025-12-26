from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from robot_controller.swerve_math import SwerveKinematics
from std_msgs.msg import Float64


class SwerveControllerNode(Node):
    """ROS 2 Node for Swerve Drive Control."""

    def __init__(self):
        super().__init__('swerve_controller')

        # Parameters
        self.declare_parameter('wheel_base', 0.5)
        self.declare_parameter('track_width', 0.5)
        self.declare_parameter('continuous_steering', True)
        self.declare_parameter('min_steering_angle', -1.57)  # Used if continuous is False
        self.declare_parameter('max_steering_angle', 1.57)   # Used if continuous is False
        self.declare_parameter('max_speed', 1.0)  # Limit individual wheel speed

        wheel_base = self.get_parameter('wheel_base').value
        track_width = self.get_parameter('track_width').value
        continuous_steering = self.get_parameter('continuous_steering').value
        min_steer = self.get_parameter('min_steering_angle').value
        max_steer = self.get_parameter('max_steering_angle').value
        self.max_speed = self.get_parameter('max_speed').value

        self.kinematics = SwerveKinematics(
            wheel_base=wheel_base,
            track_width=track_width,
            min_steering_angle=min_steer,
            max_steering_angle=max_steer,
            continuous_steering=continuous_steering
        )

        # Publishers for 4 modules (FL, FR, BL, BR)
        # 0: FL, 1: FR, 2: BL, 3: BR
        self.throttle_pubs = []
        self.steering_pubs = []

        for i in range(4):
            t_pub = self.create_publisher(Float64, f'drive_modules/{i}/throttle', 10)
            s_pub = self.create_publisher(Float64, f'drive_modules/{i}/steering', 10)
            self.throttle_pubs.append(t_pub)
            self.steering_pubs.append(s_pub)

        # Subscription
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # State for optimization (current angles)
        self.current_angles = [0.0, 0.0, 0.0, 0.0]

        self.get_logger().info('Swerve Controller Started')

    def cmd_vel_callback(self, msg):
        """Handle cmd_vel callbacks."""
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        # Calculate target states
        # Returns [(speed, angle), ...] for FL, FR, BL, BR
        raw_targets = self.kinematics.calculate_wheel_states(vx, vy, omega)

        # Optimize and publish
        for i, (target_speed, target_angle) in enumerate(raw_targets):

            # Optimize relative to current state
            final_speed, final_angle = self.kinematics.optimize_with_current_state(
                target_speed, target_angle, self.current_angles[i]
            )

            # Clamp speed
            final_speed = max(min(final_speed, self.max_speed), -self.max_speed)

            # Update state
            self.current_angles[i] = final_angle

            # Publish
            t_msg = Float64()
            t_msg.data = float(final_speed)
            self.throttle_pubs[i].publish(t_msg)

            s_msg = Float64()
            s_msg.data = float(final_angle)
            self.steering_pubs[i].publish(s_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

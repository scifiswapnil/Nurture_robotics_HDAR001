import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class SteeringMonitor(Node):
    def __init__(self):
        super().__init__('steering_monitor')
        
        # Subscribers
        for i in range(4):
            self.create_subscription(
                Float64,
                f'/drive_modules/module_{i}/steering',
                lambda msg, index=i: self.steering_callback(msg, index),
                10
            )

        self.get_logger().info('Steering Monitor Started. Listening for steering angles...')

    def steering_callback(self, msg, index):
        raw_rad = msg.data
        
        # Normalize to [0, 2pi)
        normalized_rad = raw_rad % (2 * math.pi)
        if normalized_rad < 0:
            normalized_rad += 2 * math.pi
            
        # Convert to degrees [0, 360) for display
        deg = math.degrees(normalized_rad)
        
        self.get_logger().info(
            f'Module {index}: Raw={raw_rad:.4f} rad -> Normalized={normalized_rad:.4f} rad ({deg:.2f} deg)'
        )

def main(args=None):
    rclpy.init(args=args)
    node = SteeringMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

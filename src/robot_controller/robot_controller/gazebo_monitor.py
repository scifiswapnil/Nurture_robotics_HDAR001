import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class GazeboSteeringMonitor(Node):
    def __init__(self):
        super().__init__('gazebo_steering_monitor')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        
        self.target_joints = ['FLSteer', 'FRSteer', 'BLSteer', 'BRSteer']
        self.get_logger().info(f'Gazebo Steering Monitor Started. Watching: {self.target_joints}')

    def listener_callback(self, msg):
        # Create a dictionary for quick lookup
        joint_dict = dict(zip(msg.name, msg.position))
        
        output_strs = []
        found_any = False
        
        for name in self.target_joints:
            if name in joint_dict:
                found_any = True
                raw_rad = joint_dict[name]
                
                # Normalize to [0, 2pi)
                normalized_rad = raw_rad % (2 * math.pi)
                if normalized_rad < 0:
                    normalized_rad += 2 * math.pi
                    
                deg = math.degrees(normalized_rad)
                output_strs.append(f'{name}: {deg:.2f}°')
        
        if found_any:
            self.get_logger().info(' | '.join(output_strs))

def main(args=None):
    rclpy.init(args=args)
    node = GazeboSteeringMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

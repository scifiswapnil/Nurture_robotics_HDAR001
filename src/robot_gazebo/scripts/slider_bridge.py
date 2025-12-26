#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

class SliderBridge(Node):
    def __init__(self):
        super().__init__('slider_bridge')
        
        # Target joints in order matching controllers.yaml (velocity controller)
        self.target_joints = [
            'FLDriveL', 'FLDriveR',
            'FRDriveL', 'FRDriveR',
            'BLDriveL', 'BLDriveR',
            'BRDriveL', 'BRDriveR'
        ]
        
        # P-Controller Gain
        self.kp = 5.0
        
        # State storage
        self.current_positions = {}
        self.target_positions = {}
        
        # QoS Profile for Controller (Best Effort to match subscriber)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publisher for Velocity Commands
        self.vel_pub = self.create_publisher(
            Float64MultiArray,
            '/robot_controller/commands',
            qos_profile
        )
        
        # Slider Subscription (Targets)
        self.slider_sub = self.create_subscription(
            JointState,
            'slider_states',
            self.slider_callback,
            10
        )
        
        # Joint State Subscription (Feedback)
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # Control Loop Timer (e.g., 20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('P-Control Bridge Started. Mapping Sliders -> Velocity.')

    def slider_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.target_joints:
                self.target_positions[name] = pos

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.target_joints:
                self.current_positions[name] = pos

    def control_loop(self):
        # We need both targets and current state to compute velocity
        if not self.target_positions:
            return

        command_msg = Float64MultiArray()
        velocities = []
        
        for joint in self.target_joints:
            target = self.target_positions.get(joint, 0.0)
            current = self.current_positions.get(joint, 0.0)
            
            # P-Control: error = target - current
            error = target - current
            
            # Simple Velocity Command
            vel = self.kp * error
            
            # Optional: Limit velocity
            max_vel = 10.0
            vel = max(-max_vel, min(max_vel, vel))
            
            velocities.append(vel)
            
        command_msg.data = velocities
        # Debug log every 20th tick
        # self.get_logger().info(f'Publishing {velocities}') # Uncomment if needed
        self.vel_pub.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SliderBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

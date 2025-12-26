#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import sys
import termios
import tty
import select

# Messages
msg = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d

w/s : increase/decrease linear velocity (Forward/Backward)
a/d : increase/decrease angular velocity (Turn Left/Right)
space: force stop
q   : quit

CTRL-C to quit
"""

class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        
        # Best Effort to match controller
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/robot_controller/commands',
            qos_profile
        )
        
        # Joint Configuration
        # Determine logical "Side" for Differential Drive (Left vs Right of robot)
        # Determine "Inversion" if wheels on same module are flipped
        self.joint_config = {
            # Front Left Module (Left Side)
            'FLDriveL': {'side': 'left', 'invert': True},
            'FLDriveR': {'side': 'left', 'invert': False}, 
            
            # Front Right Module (Right Side)
            'FRDriveL': {'side': 'right', 'invert': True},
            'FRDriveR': {'side': 'right', 'invert': False},
            
            # Back Left Module (Left Side)
            'BLDriveL': {'side': 'left', 'invert': True},
            'BLDriveR': {'side': 'left', 'invert': False},
            
            # Back Right Module (Right Side)
            'BRDriveL': {'side': 'right', 'invert': True},
            'BRDriveR': {'side': 'right', 'invert': False},
        }

        # Verify target_joints order matches what we expect
        self.target_joints = [
            'FLDriveL', 'FLDriveR',
            'FRDriveL', 'FRDriveR',
            'BLDriveL', 'BLDriveR',
            'BRDriveL', 'BRDriveR'
        ]
        
        # Current Speeds
        self.speed = 10.0 # rad/s
        self.turn = 5.0
        self.target_linear = 0.0
        self.target_angular = 0.0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def map_velocities(self, linear, angular):
        # Differential Drive Logic
        left_vel = linear - angular
        right_vel = linear + angular
        
        msg = Float64MultiArray()
        velocities = []

        # Iterate strictly over the target_joints list to ensure correct order
        for joint_name in self.target_joints:
            config = self.joint_config.get(joint_name)
            
            if not config:
                self.get_logger().warn(f"Joint {joint_name} not configured!")
                velocities.append(0.0)
                continue
                
            # Select Base Velocity
            if config['side'] == 'left':
                v = left_vel
            else:
                v = right_vel
                
            # Apply Inversion
            if config['invert']:
                v = -v
                
            velocities.append(v)
        
        msg.data = velocities
        return msg

def main(args=None):
    rclpy.init(args=args)
    teleop = TeleopKey()
    
    print(msg)
    
    try:
        while True:
            key = teleop.get_key()
            
            if key == 'w':
                teleop.target_linear += 1.0
            elif key == 's':
                teleop.target_linear -= 1.0
            elif key == 'a':
                teleop.target_angular += 1.0
            elif key == 'd':
                teleop.target_angular -= 1.0
            elif key == ' ':
                teleop.target_linear = 0.0
                teleop.target_angular = 0.0
            elif key == 'q' or key == '\x03':
                break
                
            # Publish
            cmd = teleop.map_velocities(teleop.target_linear, teleop.target_angular)
            teleop.pub.publish(cmd)
            
            print(f"\rLin: {teleop.target_linear} | Ang: {teleop.target_angular}", end='')
            
    except Exception as e:
        print(e)
        
    finally:
        # Stop
        cmd = teleop.map_velocities(0.0, 0.0)
        teleop.pub.publish(cmd)
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    main()

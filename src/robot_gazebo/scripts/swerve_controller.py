#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import sys
import termios
import tty
import select
import math

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class SwerveModule:
    def __init__(self, name, motor_l_name, motor_r_name, steer_name, inverted=False):
        self.name = name
        self.motor_l = motor_l_name
        self.motor_r = motor_r_name
        self.steer_joint = steer_name
        self.inverted = inverted
        
        # PID Constants for Steering
        self.kp = 8.0
        self.ki = 0.0
        self.kd = 0.0
        
        self.current_steer_angle = 0.0
        
    def update_feedback(self, angle):
        self.current_steer_angle = angle
        
    def calculate_commands(self, target_speed, target_angle, dt):
        """
        Differential Swerve Logic:
        Motor_L = Drive + Steer_Effort
        Motor_R = Drive - Steer_Effort
        (Signs depend on physical gearing, verified experimentally)
        """
        
        # Calculate Steering Error (Shortest Path)
        current = normalize_angle(self.current_steer_angle)
        target = normalize_angle(target_angle)
        error = normalize_angle(target - current)
        
        # Simple P-Controller for Steering Velocity component
        steer_vel = self.kp * error
        
        # Clamp Steering Velocity
        max_steer_vel = 5.0
        steer_vel = max(-max_steer_vel, min(max_steer_vel, steer_vel))
        
        # Mixing
        # Assuming: 
        # Positive Diff (L > R) -> CCW Steering
        # Positive Sum (L + R) -> Forward Drive
        
        v_l = target_speed - steer_vel  
        v_r = target_speed + steer_vel
        
        # Inversion (for Right side modules usually)
        if self.inverted:
            # If the module is physically mirrored, we might need to swap or invert logic
            # Based on previous tests, 'Right' modules had 'Right' motor inverted.
            # But here we output to 'DriveL' and 'DriveR'.
            # Let's keep the module logic consistent and apply motor inversion in the node mapping.
            pass

        return {self.motor_l: v_l, self.motor_r: v_r}

class SwerveTeleop(Node):
    def __init__(self):
        super().__init__('swerve_teleop')
        
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
        
        self.sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )
        
        # Define Modules 
        # (Name, MotorL, MotorR, SteerJoint)
        self.modules = [
            SwerveModule('FL', 'FLDriveL', 'FLDriveR', 'FLSteer'),
            SwerveModule('FR', 'FRDriveL', 'FRDriveR', 'FRSteer'),
            SwerveModule('BL', 'BLDriveL', 'BLDriveR', 'BLSteer'),
            SwerveModule('BR', 'BRDriveL', 'BRDriveR', 'BRSteer')
        ]
        
        # Output Joint Order (Must match controllers.yaml)
        # Based on previous controller.yaml:
        # FLDriveL, FLDriveR, FRDriveL, FRDriveR, ...
        self.joint_order = [
            'FLDriveL', 'FLDriveR',
            'FRDriveL', 'FRDriveR',
            'BLDriveL', 'BLDriveR',
            'BRDriveL', 'BRDriveR'
        ]
        
        # State
        self.target_speed = 0.0
        self.target_angle = 0.0
        self.current_commands = {j: 0.0 for j in self.joint_order}

        print("Differential Swerve Teleop")
        print("W/S: Drive Speed")
        print("A/D: Steer Angle (+/- 10 deg)")
        print("Space: Stop")
        print("Q: Quit")

    def joint_callback(self, msg):
        # Update modules with feedback
        for mod in self.modules:
            if mod.steer_joint in msg.name:
                idx = msg.name.index(mod.steer_joint)
                mod.update_feedback(msg.position[idx])

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        try:
            while True:
                rclpy.spin_once(self, timeout_sec=0.01)
                key = self.get_key()
                
                if key == 'w':
                    self.target_speed += 1.0
                elif key == 's':
                    self.target_speed -= 1.0
                elif key == 'a':
                    self.target_angle += math.radians(10)
                elif key == 'd':
                    self.target_angle -= math.radians(10)
                elif key == ' ':
                    self.target_speed = 0.0
                    self.target_angle = 0.0
                elif key == 'q' or key == '\x03':
                    break
                    
                # Update Modules
                for mod in self.modules:
                    cmds = mod.calculate_commands(self.target_speed, self.target_angle, 0.05)
                    
                    # Apply to global map
                    for j_name, cmd in cmds.items():
                        # FIX: Apply the 'Right Wheel Inversion' from Teleop Check
                        if j_name.endswith('DriveR'):
                             cmd = -cmd # Assume 180 flip
                        self.current_commands[j_name] = cmd
                        
                # Publish
                msg = Float64MultiArray()
                msg.data = [self.current_commands[j] for j in self.joint_order]
                self.pub.publish(msg)
                
                print(f"\rSpeed: {self.target_speed:.1f} | Angle: {math.degrees(self.target_angle):.1f}", end='')
                
        except Exception as e:
            print(e)
        finally:
            self.stop()

    def stop(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * len(self.joint_order)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    teleop = SwerveTeleop()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    main()


# root@robotboy-ROG-Strix-G16:/home/robotboy/nuture_robotics# ros2 control switch_controllers --activate robot_controller

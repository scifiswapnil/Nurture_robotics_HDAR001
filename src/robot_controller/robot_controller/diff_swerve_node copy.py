import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState
from robot_controller.swerve_math import SwerveKinematics
import math

class PIDController:
    def __init__(self, kp, ki, kd, min_out=None, max_out=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        self.prev_error = 0.0
        self.integral = 0.0
        
    def update(self, error, dt):
        self.integral += error * dt
        
        # Anti-windup: Clamp integral if limits are set
        if self.max_out is not None and self.min_out is not None:
             # Simple clamping of integral could be done, or clamp output
             pass

        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        if self.max_out is not None and self.min_out is not None:
            if output > self.max_out:
                output = self.max_out
                # Clamp integral to prevent further windup
                self.integral -= error * dt 
            elif output < self.min_out:
                output = self.min_out
                # Clamp integral
                self.integral -= error * dt
                
        return output

class DiffSwerveNode(Node):
    def __init__(self):
        super().__init__('diff_swerve_controller')
        
        # Params
        self.declare_parameter('wheel_base', 0.5)
        self.declare_parameter('track_width', 0.5)
        self.declare_parameter('steer_kp', 0.1)
        self.declare_parameter('steer_ki', 0.01)
        self.declare_parameter('steer_kd', 0.0)
        self.declare_parameter('max_speed', 2.0)
        
        wb = self.get_parameter('wheel_base').value
        tw = self.get_parameter('track_width').value
        kp = self.get_parameter('steer_kp').value
        ki = self.get_parameter('steer_ki').value
        kd = self.get_parameter('steer_kd').value
        self.max_speed = self.get_parameter('max_speed').value
        
        self.get_logger().info(f"Loaded Params: kp={kp}, ki={ki}, max_speed={self.max_speed}")
        
        self.kinematics = SwerveKinematics(wb, tw, continuous_steering=True)
        
        # 4 Modules: FL, FR, BL, BR
        # Motors: [Left, Right]
        # Names based on URDF snippets:
        # FL: FLDL, FLDR (Steer: FLSteer)
        # FR: FRDL, FRDR (Steer: FRSteer)
        # BL: BLDL, BLDR (Steer: BLSteer)
        # BR: BRDL, BRDR (Steer: BRSteer)
        
        self.modules = [
            {'name': 'FL', 'steer_joint': 'FLSteer', 'motors': ['FLDL', 'FLDR'], 'pid': PIDController(kp, ki, kd, -self.max_speed, self.max_speed), 'current_angle': 0.0, 'error_angle': 0.0},
            {'name': 'FR', 'steer_joint': 'FRSteer', 'motors': ['FRDL', 'FRDR'], 'pid': PIDController(kp, ki, kd, -self.max_speed, self.max_speed), 'current_angle': 0.0, 'error_angle': 0.0},
            {'name': 'BL', 'steer_joint': 'BLSteer', 'motors': ['BLDL', 'BLDR'], 'pid': PIDController(kp, ki, kd, -self.max_speed, self.max_speed), 'current_angle': 0.0, 'error_angle': 0.0},
            {'name': 'BR', 'steer_joint': 'BRSteer', 'motors': ['BRDL', 'BRDR'], 'pid': PIDController(kp, ki, kd, -self.max_speed, self.max_speed), 'current_angle': 0.0, 'error_angle': 0.0},
        ]
        
        # Publishers
        self.motor_pubs = {}
        for mod in self.modules:
            for m_name in mod['motors']:
                # Publish to /commands/{joint_name} (std_msgs/Float64)
                self.motor_pubs[m_name] = self.create_publisher(Float64, f'/commands/{m_name}', 10)
        
        # Publisher for all commands
        self.multi_pub = self.create_publisher(Float64MultiArray, '/robot_controller/commands', 10)
                
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        
        # Control Loop
        self.timer = self.create_timer(0.02, self.control_loop) # 50Hz
        
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_omega = 0.0
        

        self.get_logger().info("Differential Swerve Controller Started")

    def cmd_vel_cb(self, msg):
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        self.target_omega = msg.angular.z

    def joint_state_cb(self, msg):
        name_map = dict(zip(msg.name, msg.position))
        # print(f"FLSteer State: {name_map['FLSteer']}")
        # print(f"FRSteer State: {name_map['FRSteer']}")
        # print(f"BLSteer State: {name_map['BLSteer']}")
        # print(f"BRSteer State: {name_map['BRSteer']}")
        for mod in self.modules:
            if mod['steer_joint'] in name_map:
                # Update current angle (normalize if needed, but usually raw is fine if optimization handles it)
                # Actually, our optimization expects typical radians.
                mod['current_angle'] = name_map[mod['steer_joint']]
        
        # Debug: Print first module angle
        # if 'FLSteer' in name_map:
        #      self.get_logger().info(f"FLSteer Raw: {name_map['FLSteer']:.4f}")
                

    def control_loop(self):
        # Calculate kinematics targets
        targets = self.kinematics.calculate_wheel_states(self.target_vx, self.target_vy, self.target_omega)
        # print(f"Targets: {targets}")
        dt = 0.02
        
        # Collect all velocities for multi-array
        all_velocities = []
        print("---")
        for i, (t_speed, t_angle) in enumerate(targets):
            mod = self.modules[i]
            curr_angle = mod['current_angle']
            # print(curr_angle)
    
            # Optimize
            opt_speed, opt_angle = self.kinematics.optimize_with_current_state(t_speed, t_angle, curr_angle)
            
            # Print debug for one module to avoid spam
            # if i == 0:
            print(f"Optimized {mod['name']}: Speed={opt_speed:.2f}, Angle={opt_angle:.2f}")

            # Calculate Steering PID
            # Handle wrapping for error calculation (shortest distance)
            error = opt_angle - curr_angle
            # # Normalize error to [-pi, pi]
            # error = (error + math.pi) % (2 * math.pi) - math.pi
            
            # mod['error_angle'] = error
            # print(error)

            # print(f"Error {mod['name']}: {error}")
            
            # # PID output (Steering effort)
            # diff_vel = mod['pid'].update(error, dt)
            diff_vel = error * 0.25
            # Motor Velocities
            # Rule: FLDL and FLDR are opposite for driving.
            # Drive: Left = +V, Right = -V (or vice versa).
            # Steer: Left = +S, Right = +S (turn module).
            # print(diff_vel)
            
            v_left = (-opt_speed*2.0) + diff_vel
            v_right = (opt_speed*2.0) + diff_vel
            
            # Limit
            v_left = max(min(v_left, self.max_speed), -self.max_speed)
            v_right = max(min(v_right, self.max_speed), -self.max_speed)
            
            # Collect
            all_velocities.append(float(v_left))
            all_velocities.append(float(v_right))

            # Publish individual
            # Left
            msg_l = Float64()
            msg_l.data = float(v_left)
            # self.motor_pubs[mod['motors'][0]].publish(msg_l)
            
            # Right
            msg_r = Float64()
            msg_r.data = float(v_right)
            # self.motor_pubs[mod['motors'][1]].publish(msg_r)
        # print(f"All Velocities: {all_velocities}")
            
        # Publish multi-array
        multi_msg = Float64MultiArray()
        multi_msg.data = all_velocities
        self.multi_pub.publish(multi_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DiffSwerveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

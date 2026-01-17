import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState
from robot_controller.swerve_math import SwerveKinematics
import math
import time
import csv
import os

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

class DiffSwerveModule:
    def __init__(self, name, motors, steer_joint, pid, max_speed):
        self.name = name
        self.motors = motors # [Left, Right]
        self.steer_joint = steer_joint
        self.pid = pid
        self.max_speed = max_speed
        
        self.current_angle = 0.0
        
    def update(self, t_speed, t_angle, dt, kinematics):
        """
        Calculate motor velocities given target state.
        :return: (v_left, v_right, debug_info_dict)
        """
        # Optimize using kinematics helper
        opt_speed, opt_angle, flipped = kinematics.optimize_with_current_state(t_speed, t_angle, self.current_angle)
        
        # Calculate Error
        error = opt_angle - self.current_angle
        error = (error + math.pi) % (2 * math.pi) - math.pi
        
        # PID Steering Effort
        steer_effort = self.pid.update(error, dt)
        
        # Simple mixing: L = -Drive + Steer, R = Drive + Steer
        # NOTE: Log analysis showed steering was inverted (positive feedback).
        # We invert steer_effort here to fix it.
        steer_effort = -steer_effort
        
        # NOTE: User reported rotation is opposite. This implies drive is inverted.
        # Swapping signs of opt_speed.
        # Old: L = -Speed, R = Speed
        # New: L = Speed, R = -Speed
        v_left = -opt_speed + steer_effort
        v_right = opt_speed + steer_effort
        
        # Clamp
        v_left = max(min(v_left, self.max_speed), -self.max_speed)
        v_right = max(min(v_right, self.max_speed), -self.max_speed)
        
        debug_info = {
            'opt_speed': opt_speed,
            'opt_angle': opt_angle,
            'flipped': flipped,
            'error': error,
            'steer_effort': steer_effort
        }
        
        return v_left, v_right, debug_info

# ... (DiffSwerveNode class ...)

    def control_loop(self):
        # Calculate kinematics targets
        targets = self.kinematics.calculate_wheel_states(self.target_vx, self.target_vy, self.target_omega)
        dt = 0.02
        current_time = time.time() - self.start_time
        
        all_velocities = []
        
        for i, (t_speed, t_angle) in enumerate(targets):
            mod = self.modules[i]
            
            # Update Module
            v_left, v_right, dbg = mod.update(t_speed, t_angle, dt, self.kinematics)
            
            all_velocities.extend([float(v_left), float(v_right)])
            
            # Log Data
            # 'Time', 'Module', 'CmdVx', 'CmdVy', 'CmdOmega', 'TargSpeed', 'TargAngle', 'CurrAngle', 
            # 'OptSpeed', 'OptAngle', 'Flipped', 'Error', 'SteerEffort', 'V_Left', 'V_Right'
            self.csv_writer.writerow([
                f"{current_time:.4f}", mod.name,
                f"{self.target_vx:.2f}", f"{self.target_vy:.2f}", f"{self.target_omega:.2f}",
                f"{t_speed:.4f}", f"{t_angle:.4f}",
                f"{mod.current_angle:.4f}",
                f"{dbg['opt_speed']:.4f}", f"{dbg['opt_angle']:.4f}", dbg['flipped'],
                f"{dbg['error']:.4f}", f"{dbg['steer_effort']:.4f}",
                f"{v_left:.4f}", f"{v_right:.4f}"
            ])

            # Publish individual (Optional, for Gazebo/RosAPI)
            msg_l = Float64()
            msg_l.data = float(v_left)
            self.motor_pubs[mod.motors[0]].publish(msg_l)
            
            msg_r = Float64()
            msg_r.data = float(v_right)
            self.motor_pubs[mod.motors[1]].publish(msg_r)

class DiffSwerveNode(Node):
    def __init__(self):
        super().__init__('diff_swerve_controller')
        
        # Params
        self.declare_parameter('wheel_base', 0.5)
        self.declare_parameter('track_width', 0.5)
        self.declare_parameter('steer_kp', 2.5) # Increased default based on simple node experience
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
        self.modules = [
            DiffSwerveModule('FL', ['FLDL', 'FLDR'], 'FLSteer', PIDController(kp, ki, kd), self.max_speed),
            DiffSwerveModule('FR', ['FRDL', 'FRDR'], 'FRSteer', PIDController(kp, ki, kd), self.max_speed),
            DiffSwerveModule('BL', ['BLDL', 'BLDR'], 'BLSteer', PIDController(kp, ki, kd), self.max_speed),
            DiffSwerveModule('BR', ['BRDL', 'BRDR'], 'BRSteer', PIDController(kp, ki, kd), self.max_speed),
        ]
        
        # Publishers
        self.motor_pubs = {}
        for mod in self.modules:
            for m_name in mod.motors:
                self.motor_pubs[m_name] = self.create_publisher(Float64, f'/commands/{m_name}', 10)
        
        self.multi_pub = self.create_publisher(Float64MultiArray, '/robot_controller/commands', 10)
        self.vis_pub = self.create_publisher(Float64MultiArray, '/swerve_state', 10)
                
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        
        # Control Loop
        self.timer = self.create_timer(0.02, self.control_loop) # 50Hz
        
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_omega = 0.0
        
        # Logging Setup
        log_dir = "/home/robotboy/nuture_robotics/logs"
        os.makedirs(log_dir, exist_ok=True)
        self.log_file_path = os.path.join(log_dir, f"swerve_debug_{int(time.time())}.csv")
        self.log_file = open(self.log_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        # Header
        self.csv_writer.writerow([
            'Time', 'Module', 
            'CmdVx', 'CmdVy', 'CmdOmega', 
            'TargSpeed', 'TargAngle', 
            'CurrAngle', 
            'OptSpeed', 'OptAngle', 'Flipped', 
            'Error', 'SteerEffort', 
            'V_Left', 'V_Right'
        ])
        self.start_time = time.time()
        
        self.get_logger().info(f"Differential Swerve Controller Started (Ported Logic). Log: {self.log_file_path}")

    def cmd_vel_cb(self, msg):
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        self.target_omega = msg.angular.z

    def joint_state_cb(self, msg):
        name_map = dict(zip(msg.name, msg.position))
        for mod in self.modules:
            if mod.steer_joint in name_map:
                mod.current_angle = -name_map[mod.steer_joint]

    def control_loop(self):
        # Calculate kinematics targets
        targets = self.kinematics.calculate_wheel_states(self.target_vx, self.target_vy, self.target_omega)
        dt = 0.02
        current_time = time.time() - self.start_time
        
        all_velocities = []
        
        for i, (t_speed, t_angle) in enumerate(targets):
            mod = self.modules[i]
            
            # Update Module
            v_left, v_right, dbg = mod.update(t_speed, t_angle, dt, self.kinematics)
            
            all_velocities.extend([float(v_left), float(v_right)])
            
            # Log Data
            # 'Time', 'Module', 'CmdVx', 'CmdVy', 'CmdOmega', 'TargSpeed', 'TargAngle', 'CurrAngle', 
            # 'OptSpeed', 'OptAngle', 'Flipped', 'Error', 'SteerEffort', 'V_Left', 'V_Right'
            self.csv_writer.writerow([
                f"{current_time:.4f}", mod.name,
                f"{self.target_vx:.2f}", f"{self.target_vy:.2f}", f"{self.target_omega:.2f}",
                f"{t_speed:.4f}", f"{t_angle:.4f}",
                f"{mod.current_angle:.4f}",
                f"{dbg['opt_speed']:.4f}", f"{dbg['opt_angle']:.4f}", dbg['flipped'],
                f"{dbg['error']:.4f}", f"{dbg['steer_effort']:.4f}",
                f"{v_left:.4f}", f"{v_right:.4f}"
            ])

            # Publish individual (Optional, for Gazebo/RosAPI)
            msg_l = Float64()
            msg_l.data = float(v_left)
            self.motor_pubs[mod.motors[0]].publish(msg_l)
            
            msg_r = Float64()
            msg_r.data = float(v_right)
            self.motor_pubs[mod.motors[1]].publish(msg_r)
            
        # Publish multi-array
        multi_msg = Float64MultiArray()
        multi_msg.data = all_velocities
        self.multi_pub.publish(multi_msg)
        
        # Publish Visualization State
        # [Ang, VL, VR] per module
        vis_state = []
        for mod, (v_left, v_right) in zip(self.modules, zip(all_velocities[::2], all_velocities[1::2])):
             vis_state.extend([mod.current_angle, v_left, v_right])
             
        vis_msg = Float64MultiArray()
        vis_msg.data = vis_state
        self.vis_pub.publish(vis_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DiffSwerveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

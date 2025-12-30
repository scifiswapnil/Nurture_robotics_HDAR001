
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import math
import numpy as np

class SwerveVisualizer(Node):
    def __init__(self):
        super().__init__('swerve_visualizer')
        self.create_subscription(Float64MultiArray, '/swerve_state', self.state_cb, 10)
        
        # State: [FL_Ang, FL_VL, FL_VR, FR_Ang, ... BL ..., BR ...]
        self.current_state = [0.0] * 12
        
        self.lock = threading.Lock()
        self.get_logger().info("Swerve Visualizer Started")

    def state_cb(self, msg):
        with self.lock:
            self.current_state = msg.data

def main(args=None):
    rclpy.init(args=args)
    vis_node = SwerveVisualizer()
    
    # Run ROS in a thread
    thread = threading.Thread(target=rclpy.spin, args=(vis_node,), daemon=True)
    thread.start()
    
    # Matplotlib Setup
    fig, ax = plt.subplots(figsize=(8, 8))
    plt.title("Swerve Drive Visualization")
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_aspect('equal')
    ax.grid(True)
    
    # Robot Body
    base_rect = plt.Rectangle((-0.4, -0.4), 0.8, 0.8, fill=False, edgecolor='blue', lw=2)
    ax.add_patch(base_rect)
    
    # Wheel Positions (FL, FR, BL, BR) - approximate for visualization
    positions = [
        (0.4, 0.4),   # FL
        (0.4, -0.4),  # FR
        (-0.4, 0.4),  # BL
        (-0.4, -0.4)  # BR
    ]
    
    names = ["FL", "FR", "BL", "BR"]
    
    # Graphics objects
    wheels = []
    arrows_l = []
    arrows_r = []
    arrows_main = []
    texts = []
    
    WHEEL_SEP = 0.05 # Visual separation distance from center line
    
    for i, (x, y) in enumerate(positions):
        # Wheel representation (Line)
        line, = ax.plot([], [], 'k-', lw=5)
        # Direction arrows (Left and Right)
        # Initialize off-screen/zero
        al = ax.arrow(x, y, 0, 0, head_width=0.03, head_length=0.05, fc='b', ec='b')
        ar = ax.arrow(x, y, 0, 0, head_width=0.03, head_length=0.05, fc='g', ec='g')
        # Overall Velocity Arrow (Center)
        am = ax.arrow(x, y, 0, 0, head_width=0.04, head_length=0.08, fc='k', ec='k', alpha=0.5)
        
        # Stats text
        txt = ax.text(x, y - 0.2, f"{names[i]}\nL: 0.0\nR: 0.0", ha='center')
        
        wheels.append(line)
        arrows_l.append(al)
        arrows_r.append(ar)
        arrows_main.append(am)
        texts.append(txt)
        
    def update(frame):
        with vis_node.lock:
            data = vis_node.current_state[:]
            
        if len(data) < 12:
            return wheels + arrows_l + arrows_r + arrows_main + texts
            
        for i in range(4):
            base_idx = i * 3
            angle = data[base_idx]
            vl = data[base_idx + 1]
            vr = data[base_idx + 2]
            
            cx, cy = positions[i]
            
            # 1. Update Wheel Line
            # Visual Length = 0.2
            dx = 0.1 * math.cos(angle)
            dy = 0.1 * math.sin(angle)
            wheels[i].set_data([cx - dx, cx + dx], [cy - dy, cy + dy])
            
            # 2. Update Arrows (Velocity vectors)
            # Calculate axle vector (perpendicular to steering)
            # axle x = -sin, axle y = cos
            ax_x = -math.sin(angle)
            ax_y = math.cos(angle)
            
            # Left Arrow Pos
            lx = cx + (ax_x * WHEEL_SEP)
            ly = cy + (ax_y * WHEEL_SEP)
            
            # Right Arrow Pos
            rx = cx - (ax_x * WHEEL_SEP)
            ry = cy - (ax_y * WHEEL_SEP)
            
            # Remove old arrows
            arrows_l[i].remove()
            arrows_r[i].remove()
            arrows_main[i].remove()
            
            # New Arrows
            # Scale factor for visualization
            SCALE = 0.1
            
            # Left Arrow
            dx_l = vl * SCALE * math.cos(angle)
            dy_l = vl * SCALE * math.sin(angle)
            color_l = 'r' if vl < 0 else 'g'
            arrows_l[i] = ax.arrow(lx, ly, dx_l, dy_l, head_width=0.03, color=color_l)

            # Right Arrow
            dx_r = vr * SCALE * math.cos(angle)
            dy_r = vr * SCALE * math.sin(angle)
            color_r = 'r' if vr < 0 else 'g'
            arrows_r[i] = ax.arrow(rx, ry, dx_r, dy_r, head_width=0.03, color=color_r)
            
            # Overall Arrow (Center)
            # Drive velocity = (Right - Left) / 2.0 (Since Left is inverted for drive)
            # Reconstruct drive speed from differential signals
            v_drive = (vr - vl) / 2.0
            
            dx_m = v_drive * SCALE * math.cos(angle)
            dy_m = v_drive * SCALE * math.sin(angle)
            # Make it slightly transparent or distinct
            arrows_main[i] = ax.arrow(cx, cy, dx_m, dy_m, head_width=0.04, color='orange', alpha=0.8)
            
            # 3. Update Text
            texts[i].set_text(f"{names[i]}\nAng: {math.degrees(angle):.0f}\nL: {vl:.2f}\nR: {vr:.2f}")
            
        return wheels + arrows_l + arrows_r + arrows_main + texts

    ani = FuncAnimation(fig, update, interval=50) # 20Hz
    plt.show()

    vis_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import numpy as np
import math

class SwerveVisualizer(Node):
    def __init__(self):
        super().__init__('swerve_visualizer')
        
        # State: [steering, throttle] for 4 modules
        # 0: FL, 1: FR, 2: BL, 3: BR
        self.module_states = [{'steering': 0.0, 'throttle': 0.0} for _ in range(4)]
        
        # Subscribers
        self.subs = []
        for i in range(4):
            # Capture i in lambda
            self.create_module_subscription(i)
            
        self.get_logger().info('Swerve Visualizer Started')
        
        # Robot dimensions for visualization (normalized or approx)
        self.L = 0.5
        self.W = 0.5
        
        # Positions
        # FL, FR, BL, BR
        self.positions = [
            (self.L/2, self.W/2),   # FL
            (self.L/2, -self.W/2),  # FR
            (-self.L/2, self.W/2),  # BL
            (-self.L/2, -self.W/2)  # BR
        ]

    def create_module_subscription(self, index):
        # Steering
        self.create_subscription(
            Float64,
            f'/drive_modules/module_{index}/steering',
            lambda msg: self.steering_callback(msg, index),
            10
        )
        # Throttle
        self.create_subscription(
            Float64,
            f'/drive_modules/module_{index}/throttle',
            lambda msg: self.throttle_callback(msg, index),
            10
        )

    def steering_callback(self, msg, index):
        self.module_states[index]['steering'] = msg.data

    def throttle_callback(self, msg, index):
        self.module_states[index]['throttle'] = msg.data

    def update_plot(self, ax, quiver_list):
        # Update arrows
        for i, q in enumerate(quiver_list):
            state = self.module_states[i]
            x, y = self.positions[i]
            
            # Vector components
            # Length proportional to throttle
            length = state['throttle']
            angle = state['steering']
            
            u = length * math.cos(angle)
            v = length * math.sin(angle)
            
            # If throttle is 0, we can still show direction but small?
            # Or just show the vector exactly as is.
            
            q.set_UVC(u, v)

def main(args=None):
    rclpy.init(args=args)
    node = SwerveVisualizer()
    
    plt.ion()
    fig, ax = plt.subplots()
    
    # Setup plot
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_title('Swerve Drive System State')
    
    # Draw chassis
    # FL -> FR -> BR -> BL -> FL
    chassis_x = [node.positions[0][0], node.positions[1][0], node.positions[3][0], node.positions[2][0], node.positions[0][0]]
    chassis_y = [node.positions[0][1], node.positions[1][1], node.positions[3][1], node.positions[2][1], node.positions[0][1]]
    ax.plot(chassis_x, chassis_y, 'k-', lw=2)
    
    # Initialize quivers (arrows)
    quivers = []
    colors = ['r', 'g', 'b', 'm']
    labels = ['FL', 'FR', 'BL', 'BR']
    
    for i in range(4):
        x, y = node.positions[i]
        # Initial vector
        q = ax.quiver(x, y, 0, 0, color=colors[i], angles='xy', scale_units='xy', scale=1)
        quivers.append(q)
        ax.text(x, y, labels[i], fontsize=12)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.update_plot(ax, quivers)
            fig.canvas.draw()
            fig.canvas.flush_events()
            # plt.pause(0.01) # pause can be slow, flush_events is better with spin_once loop
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class ChenilleNode(Node):
    def __init__(self):
        super().__init__('chenille_node')
        
        # Publisher pour publier des commandes Twist
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Subscriber pour écouter le joystick
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Paramètres modifiables facilement
        self.enable_button_index = 5  # bouton RB pour activer (comme teleop_twist_joy)
        self.require_enable_button = False  # toujours actif
        self.scale_linear = 1.0
        self.scale_angular = 1.5
        self.get_logger().info("Chenille node started, listening to joystick...")

    def joy_callback(self, joy_msg):
        twist = Twist()
        
        # Axes pour le contrôle des chenilles
        left_stick = joy_msg.axes[4]   # Stick gauche vertical
        right_stick = joy_msg.axes[1]  # Stick droit vertical

        # Activation selon bouton si nécessaire
        enabled = True
        if self.require_enable_button:
            enabled = (joy_msg.buttons[self.enable_button_index] == 1)

        if enabled:
            #self.get_logger().info(f"left_stick = {left_stick}")
            # Pilotage type “char”
            # Avance/recul : moyenne des sticks
            twist.linear.x = (left_stick + right_stick) / 2.0 * self.scale_linear
            # Rotation : différence des sticks
            twist.angular.z = (left_stick - right_stick) / 2.0 * self.scale_angular
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Arrêt d'urgence sur bouton LB
        if joy_msg.buttons[4] == 1:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Arrêt d'urgence !")

        # Publie le message
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ChenilleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

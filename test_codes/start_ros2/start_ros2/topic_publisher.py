#!/usr/bin/env python3
# Authors: Eungi Cho

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


class TopicPublisher(Node):

    def __init__(self):
        # Starts a new node
        super().__init__('custom_teleop_turtle')

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)

        self.velocity_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', QOS_RKL10V)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_move_topic)

    def publish_move_topic(self):
        vel_msg = Twist()

        # Receiveing the user's input
        print("Let's move your robot")
        speed = float(input("Input your speed:"))
        distance = float(input("Type your distance:"))
        isForward = bool(input("Foward?: "))

        # Checking if the movement is forward or backwards
        if(isForward):
            vel_msg.linear.x = abs(speed)
        else:
            vel_msg.linear.x = -abs(speed)
        # Since we are moving just in x-axis
        vel_msg.linear.y = 0.
        vel_msg.linear.z = 0.
        vel_msg.angular.x = 0.
        vel_msg.angular.y = 0.
        vel_msg.angular.z = 0.

        self.velocity_publisher.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        topic_publisher = TopicPublisher()
        try:
            rclpy.spin(topic_publisher)
        except KeyboardInterrupt:
            topic_publisher.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            topic_publisher.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

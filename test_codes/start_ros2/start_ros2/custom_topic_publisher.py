#!/usr/bin/env python3
# Authors: Eungi Cho

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from custom_msgs.msg import Num


class CustomTopicPublisher(Node):

    def __init__(self):
        # Starts a new node
        super().__init__('custom_topic_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)

        self.custom_topic_publisher = self.create_publisher(Num, 'custom_topic', qos_profile)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_custom_topic)

        self.index = 0

    def publish_custom_topic(self):
        custom_msg = Num()
        custom_msg.num = self.index
        self.custom_topic_publisher.publish(custom_msg)

        print('[Publisher] topic: {0}'.format(self.index))
        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    try:
        custom_topic_publisher = CustomTopicPublisher()
        try:
            rclpy.spin(custom_topic_publisher)
        except KeyboardInterrupt:
            custom_topic_publisher.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            custom_topic_publisher.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

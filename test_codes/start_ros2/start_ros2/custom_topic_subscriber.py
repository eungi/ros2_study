#!/usr/bin/env python3
# Authors: Eungi Cho

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from custom_msgs.msg import Num


class CustomTopicSubscriber(Node):

    def __init__(self):
        # Starts a new node
        super().__init__('custom_topic_subscriber')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)

        self.custom_topic_subscriber = self.create_subscription(
            Num,
            'custom_topic',
            self.subscribe_custom_topic,
            qos_profile)

    def subscribe_custom_topic(self, msg):
        print('[Subscriber] topic: {0}'.format(msg.num))


def main(args=None):
    rclpy.init(args=args)
    try:
        custom_topic_subscriber = CustomTopicSubscriber()
        try:
            rclpy.spin(custom_topic_subscriber)
        except KeyboardInterrupt:
            custom_topic_subscriber.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            custom_topic_subscriber.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

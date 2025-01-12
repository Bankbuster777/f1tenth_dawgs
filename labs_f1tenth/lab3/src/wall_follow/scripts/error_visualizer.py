#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Float64

import matplotlib.pyplot as plt
import numpy as np

class ErrorVisualizer(Node):
    """
    Subscribe error topic and visualize it with graph.
    """
    def __init__(self):
        super().__init__('error_visualizer_node')
        qos_profile = QoSProfile(depth=10)
        error_topic = '/error'

        self.error_subscriber_ = self.create_subscription(Float64, error_topic, self.callback_error, qos_profile)

    def callback_error(self):
        error_msg = Float64()
        self.get_logger().info('Test message for the first time', once = True)






def main(args=None):
    rclpy.init(args=args)
    print("ErrorVisualizer Initialized")
    error_visualizer_node = ErrorVisualizer()
    rclpy.spin(error_visualizer_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    error_visualizer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
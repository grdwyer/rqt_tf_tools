import os
import subprocess
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.time import Duration

from rclpy.node import Node
from time import sleep

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from tf2_ros import TransformListener, Buffer, LookupException


class Listener(Node):
    def __init__(self):
        super(Listener, self).__init__("tf_listener")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1, self.run)

    def run(self):
        self.get_logger().info(self.tf_buffer.all_frames_as_yaml())


def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

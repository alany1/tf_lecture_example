import numpy as np
import rclpy
import tf2_ros
import time
from rclpy.node import Node

from .utils import se3_to_tf


class Planet(Node):

    def __init__(self):
        super().__init__('planet')

        timer_period = 1.0 / 20  # hz
        self.timer = self.create_timer(timer_period, self.node_callback)

        self.t = time.perf_counter()
        self.rotation_rate = 0.1  # Hz
        self.rotation_radius = 5.0  # meters

        self.br = tf2_ros.TransformBroadcaster(self)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

    def node_callback(self):
        new_t = time.perf_counter()
        time_elapsed = new_t - self.t

        angle = 2 * np.pi * self.rotation_rate * time_elapsed

        planet_to_sun_translation = [self.rotation_radius * np.cos(angle), self.rotation_radius * np.sin(angle), 0]
        planet_to_sun_translation = np.array(planet_to_sun_translation).T

        planet_to_sun_tf = np.eye(4)
        planet_to_sun_tf[:3, -1] = planet_to_sun_translation[:3]

        now = self.get_clock().now()

        planet_to_sun_msg = se3_to_tf(planet_to_sun_tf, now, parent='sun', child='planet')
        self.br.sendTransform([planet_to_sun_msg])

        self.get_logger().info('Published')


def main(args=None):
    rclpy.init(args=args)

    node = Planet()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import numpy as np
import rclpy
import tf2_ros
import time
from rclpy.node import Node

from .utils import se3_to_tf, tf_to_se3


class Satellite(Node):

    def __init__(self):
        super().__init__('satellite')

        timer_period = 1.0 / 40  # hz
        self.timer = self.create_timer(timer_period, self.node_callback)

        self.t = time.perf_counter()
        self.rotation_rate = 1  # Hz
        self.rotation_radius = 1.0  # meters

        self.br = tf2_ros.TransformBroadcaster(self)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

    def node_callback(self):
        try:
            planet_to_sun_msg: TransformStamped = self.tfBuffer.lookup_transform('sun', 'planet',
                                                                                 rclpy.time.Time())
        except tf2_ros.TransformException:
            self.get_logger().info('waiting on parent')
            return
            
        new_t = time.perf_counter()
        time_elapsed = new_t - self.t

        angle = 2 * np.pi * self.rotation_rate * time_elapsed

        satellite_to_planet_trans = [self.rotation_radius * np.cos(angle), self.rotation_radius * np.sin(angle), 0]
        satellite_to_planet_trans = np.array(satellite_to_planet_trans).T

        satellite_to_planet_tf = np.eye(4)
        satellite_to_planet_tf[:3, -1] = satellite_to_planet_trans[:3]

        planet_to_sun_tf = tf_to_se3(planet_to_sun_msg.transform)

        satellite_to_sun_tf = planet_to_sun_tf @ satellite_to_planet_tf

        now = self.get_clock().now()

        satellite_to_sun_msg = se3_to_tf(satellite_to_sun_tf, now, parent='sun', child='satellite')
        self.br.sendTransform([satellite_to_sun_msg])

        self.get_logger().info('Published')


def main(args=None):
    rclpy.init(args=args)

    node = Satellite()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

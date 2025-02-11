import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .utils import se3_to_tf


class Sun(Node):

    def __init__(self):
        super().__init__('sun')

        # broadcaster that will publish the transform
        self.static_br = StaticTransformBroadcaster(self)

        # say the sun is identity rotation, but 5 units above world origin.
        sun_to_world = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 5],
            [0, 0, 0, 1]
        ]).astype(float)

        self.make(sun_to_world)

    def make(self, mat):
        msg = se3_to_tf(mat, self.get_clock().now(), 'world', 'sun')
        self.static_br.sendTransform(msg)

        self.get_logger().info("Published sun transform.")


def main(args=None):
    rclpy.init(args=args)

    node = Sun()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

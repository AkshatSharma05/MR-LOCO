#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',  # Change topic name if needed
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Imu):
        self.get_logger().info(
            f'\nOrientation -> x: {msg.orientation.x:.3f}, y: {msg.orientation.y:.3f}, z: {msg.orientation.z:.3f}, w: {msg.orientation.w:.3f}'
            f'\nAngular Velocity -> x: {msg.angular_velocity.x:.3f}, y: {msg.angular_velocity.y:.3f}, z: {msg.angular_velocity.z:.3f}'
            f'\nLinear Acceleration -> x: {msg.linear_acceleration.x:.3f}, y: {msg.linear_acceleration.y:.3f}, z: {msg.linear_acceleration.z:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()
    rclpy.spin(imu_subscriber)

    imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

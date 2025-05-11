#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math 

class MrLoco(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',  
            self.imu_callback,
            10)
        
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        self.kp = 1.0
        self.ki = 0.1
        self.kd = 1.0

        self.prev_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.last_time = self.get_clock().now()

    def imu_callback(self, msg: Imu):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        
        roll, _, _ = self.quaternion_to_euler(x, y, z, w)
        
        # print(
        #     f'\nOrientation -> x: {pitch:.2f}, y: {roll:.2f}, z: {yaw:.2f}'
        # )


        error = 0.0 - roll

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        self.integral += error * dt
        self.derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        correction = self.kp * error + self.ki * self.integral + self.kd * self.derivative
        # correction = max(min(correction, 4.0), -4.0) 

        print(
            f'\n error -> x: {error}'
            f'\n integral -> x: {self.integral}'
            f'\n derivative -> x: {self.derivative}'
            f'\n Correction -> x: {correction}'
        )

        # print(f"Error: {error}, Prev error: {self.prev_error}, dt: {dt}, Derivative: {self.derivative}")
        # print(f"Current Time: {current_time.nanoseconds / 1e9:.6f}s, "
        #     f"Last Time: {self.last_time.nanoseconds / 1e9:.6f}s, "
        #     f"dt: {dt:.6f}s")

        # if abs(error) < 0.01:
        #     correction = 0.0

        cmd = Twist()

        cmd.linear.x = -correction
        self.velocity_publisher.publish(cmd)

    def quaternion_to_euler(self, x, y, z, w):
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw




def main(args=None):
    rclpy.init(args=args)
    loco_node = MrLoco()
    rclpy.spin(loco_node)

    loco_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

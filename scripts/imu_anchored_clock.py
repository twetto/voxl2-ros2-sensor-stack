#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


class ImuAnchoredClock(Node):
    def __init__(self, imu_topic, playback_rate, publish_rate):
        super().__init__('voxl_bag_clock')
        self._playback_rate = playback_rate
        self._anchor_ros_ns = None
        self._anchor_wall_ns = None

        self._clock_pub = self.create_publisher(Clock, '/clock', 10)
        self._imu_sub = self.create_subscription(
            Imu, imu_topic, self._imu_callback, qos_profile_sensor_data)
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_clock)

        self.get_logger().info(
            'Waiting for the first IMU timestamp on {} (playback rate {}x)'.format(
                imu_topic, playback_rate))

    def _imu_callback(self, msg):
        if self._anchor_ros_ns is not None:
            return

        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns <= 0:
            self.get_logger().warning('Ignoring a non-positive IMU timestamp')
            return

        self._anchor_ros_ns = stamp_ns
        self._anchor_wall_ns = time.monotonic_ns()
        self.get_logger().info(
            'Anchored /clock at {}.{:09d}'.format(
                msg.header.stamp.sec, msg.header.stamp.nanosec))

    def _publish_clock(self):
        if self._anchor_ros_ns is None:
            return

        elapsed_wall_ns = time.monotonic_ns() - self._anchor_wall_ns
        clock_ns = self._anchor_ros_ns + int(elapsed_wall_ns * self._playback_rate)

        msg = Clock()
        msg.clock.sec = clock_ns // 1_000_000_000
        msg.clock.nanosec = clock_ns % 1_000_000_000
        self._clock_pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(
        description='Publish simulated ROS time anchored to the first replayed IMU stamp.')
    parser.add_argument('--imu-topic', default='/voxl/raw_imu')
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--publish-rate', type=float, default=200.0)
    args = parser.parse_args()

    if args.rate <= 0.0:
        parser.error('--rate must be positive')
    if args.publish_rate <= 0.0:
        parser.error('--publish-rate must be positive')

    rclpy.init(args=[])
    node = ImuAnchoredClock(args.imu_topic, args.rate, args.publish_rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

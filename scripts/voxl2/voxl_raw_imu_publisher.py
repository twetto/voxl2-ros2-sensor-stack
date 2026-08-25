#!/usr/bin/env python3

import os
import time
import struct
import ctypes

import rclpy
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import Imu

# ============================================================
# Configuration
# ============================================================

PIPE_NAME = b"imu_apps"
CLIENT_NAME = b"python_imu_ros2_fast"

CHANNEL = 0

READ_SIZE = 131072

TOPIC_NAME = "/voxl/raw_imu"
FRAME_ID = "imu_link"


# ============================================================
# imu_data_t
# ============================================================

IMU_STRUCT = struct.Struct("@I3f3ffQ")

PACKET_SIZE = IMU_STRUCT.size


class VoxlImuPublisher(Node):

    def __init__(self):

        super().__init__("voxl_raw_imu_publisher")

        # ====================================================
        # QoS
        #
        # IMPORTANT:
        #
        # depth=1 is too small for burst publishing.
        #
        # Use large queue to reduce sample overwrite.
        # ====================================================

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2000,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.publisher_ = self.create_publisher(Imu, TOPIC_NAME, qos)

        # ----------------------------------------------------
        # Reuse ROS message object
        # ----------------------------------------------------

        self.msg = Imu()

        self.msg.header.frame_id = FRAME_ID

        self.msg.orientation_covariance[0] = -1.0

        # ----------------------------------------------------
        # libmodal_pipe
        # ----------------------------------------------------

        self.lib = ctypes.CDLL("libmodal_pipe.so")

        self.lib.pipe_client_open.argtypes = [
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.c_int,
        ]

        self.lib.pipe_client_open.restype = ctypes.c_int

        self.lib.pipe_client_get_fd.argtypes = [ctypes.c_int]

        self.lib.pipe_client_get_fd.restype = ctypes.c_int

        self.lib.pipe_client_close.argtypes = [ctypes.c_int]

        self.lib.pipe_client_close.restype = None

        # ----------------------------------------------------
        # Open imu_apps
        # ----------------------------------------------------

        ret = self.lib.pipe_client_open(CHANNEL, PIPE_NAME, CLIENT_NAME, 0, READ_SIZE)

        if ret != 0:

            raise RuntimeError("pipe_client_open failed: {}".format(ret))

        # ----------------------------------------------------
        # File descriptor
        # ----------------------------------------------------

        self.fd = self.lib.pipe_client_get_fd(CHANNEL)

        if self.fd < 0:

            self.lib.pipe_client_close(CHANNEL)

            raise RuntimeError("pipe_client_get_fd failed")

        # ----------------------------------------------------
        # Partial packet buffer
        # ----------------------------------------------------

        self.remaining = b""

        # ----------------------------------------------------
        # Timestamp conversion
        # ----------------------------------------------------

        self.time_offset_ns = int((time.time() - time.monotonic()) * 1000000000.0)

        # ----------------------------------------------------
        # Statistics
        # ----------------------------------------------------

        self.total_received = 0
        self.total_published = 0

        self.start_time = time.monotonic()

        # ----------------------------------------------------
        # Start message
        # ----------------------------------------------------

        self.get_logger().info("Direct source: imu_apps")

        self.get_logger().info("imu_data_t packet size: {} bytes".format(PACKET_SIZE))

        self.get_logger().info("Publishing: {}".format(TOPIC_NAME))

        self.get_logger().info("QoS: BEST_EFFORT KEEP_LAST depth=2000")

    # ========================================================
    # Main
    # ========================================================

    def run(self):

        fd = self.fd

        packet_size = PACKET_SIZE

        unpack_from = IMU_STRUCT.unpack_from

        publish = self.publisher_.publish

        msg = self.msg

        time_offset_ns = self.time_offset_ns

        remaining = b""

        received_count = 0
        published_count = 0

        try:

            while rclpy.ok():

                # =============================================
                # Read MPA
                # =============================================

                new_data = os.read(fd, READ_SIZE)

                if not new_data:
                    continue

                # =============================================
                # Reassemble incomplete packet
                # =============================================

                if remaining:

                    data = remaining + new_data

                else:

                    data = new_data

                data_len = len(data)

                complete_bytes = data_len - (data_len % packet_size)

                if complete_bytes < data_len:

                    remaining = data[complete_bytes:]

                else:

                    remaining = b""

                # =============================================
                # Parse packet batch
                # =============================================

                offset = 0

                while offset < complete_bytes:

                    magic, ax, ay, az, gx, gy, gz, temp, timestamp_ns = unpack_from(
                        data, offset
                    )

                    offset += packet_size

                    received_count += 1

                    # =========================================
                    # Timestamp
                    # =========================================

                    ros_ns = timestamp_ns + time_offset_ns

                    sec = ros_ns // 1000000000

                    nanosec = ros_ns % 1000000000

                    msg.header.stamp.sec = int(sec)
                    msg.header.stamp.nanosec = int(nanosec)

                    # =========================================
                    # Gyroscope
                    # =========================================

                    msg.angular_velocity.x = gx
                    msg.angular_velocity.y = gy
                    msg.angular_velocity.z = gz

                    # =========================================
                    # Acceleration
                    # =========================================

                    msg.linear_acceleration.x = ax
                    msg.linear_acceleration.y = ay
                    msg.linear_acceleration.z = az

                    # =========================================
                    # Publish
                    # =========================================

                    publish(msg)

                    published_count += 1

        except KeyboardInterrupt:

            pass

        finally:

            self.remaining = remaining

            self.total_received = received_count
            self.total_published = published_count

            self.close()

    # ========================================================
    # Close
    # ========================================================

    def close(self):

        elapsed = time.monotonic() - self.start_time

        try:

            self.lib.pipe_client_close(CHANNEL)

        except Exception:

            pass

        print()

        print("Received packets : {}".format(self.total_received))

        print("Published calls  : {}".format(self.total_published))

        print("Elapsed          : {:.3f} s".format(elapsed))

        if elapsed > 0:

            print("Receive rate    : {:.1f} Hz".format(self.total_received / elapsed))

            print("Publish rate    : {:.1f} Hz".format(self.total_published / elapsed))


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = VoxlImuPublisher()

        node.run()

    except KeyboardInterrupt:

        pass

    except Exception as e:

        print("ERROR: {}".format(e))

    finally:

        if node is not None:

            node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()

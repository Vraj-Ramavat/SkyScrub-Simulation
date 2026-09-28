#!/usr/bin/env python3

import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)

from sensor_msgs.msg import LaserScan
from px4_msgs.msg import VehicleLocalPosition


DESIRED_STANDOFF_M = 2.0


class SkyScrubStandoffMonitor(Node):

    def __init__(self):
        super().__init__('skyscrub_standoff_monitor')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.range_m = None
        self.position = None

        self.create_subscription(
            LaserScan,
            '/skyscrub/front/range',
            self.range_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            px4_qos,
        )

        self.create_timer(0.5, self.report)

        self.get_logger().info('SkyScrub V0.3 corrected standoff monitor started')
        self.get_logger().info(
            f'Desired facade standoff: {DESIRED_STANDOFF_M:.2f} m'
        )
        self.get_logger().info('MONITOR ONLY — no flight commands published')

    def range_callback(self, msg):
        valid = [
            r for r in msg.ranges
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]

        if valid:
            self.range_m = statistics.median(valid)

    def position_callback(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.position = msg

    def report(self):
        if self.range_m is None:
            self.get_logger().warn('Waiting for facade range...')
            return

        if self.position is None:
            self.get_logger().warn('Waiting for PX4 local position...')
            return

        error = self.range_m - DESIRED_STANDOFF_M

        heading = self.position.heading

        forward_north = math.cos(heading)
        forward_east = math.sin(heading)

        target_x = self.position.x + error * forward_north
        target_y = self.position.y + error * forward_east

        if error > 0.05:
            action = 'MOVE FORWARD TOWARD FACADE'
        elif error < -0.05:
            action = 'MOVE BACKWARD AWAY FROM FACADE'
        else:
            action = 'HOLD STANDOFF'

        self.get_logger().info(
            '\n'
            f'Range         : {self.range_m:7.3f} m\n'
            f'Setpoint      : {DESIRED_STANDOFF_M:7.3f} m\n'
            f'Range error   : {error:+7.3f} m\n'
            f'PX4 x (North) : {self.position.x:+7.3f} m\n'
            f'PX4 y (East)  : {self.position.y:+7.3f} m\n'
            f'PX4 z (Down)  : {self.position.z:+7.3f} m\n'
            f'Heading       : {heading:+7.3f} rad\n'
            f'Heading valid : {self.position.heading_good_for_control}\n'
            f'Forward NED   : [{forward_north:+.3f}, {forward_east:+.3f}]\n'
            f'Target x*     : {target_x:+7.3f} m\n'
            f'Target y*     : {target_y:+7.3f} m\n'
            f'Action*       : {action}\n'
            '*calculation only — NO flight command sent'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SkyScrubStandoffMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

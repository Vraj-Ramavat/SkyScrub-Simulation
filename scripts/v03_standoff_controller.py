#!/usr/bin/env python3

import math
import statistics
import time

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

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


DESIRED_RANGE = 2.00

DEADBAND = 0.08

KP_RANGE = 0.25

MAX_APPROACH_SPEED = 0.20
MAX_RETREAT_SPEED = 0.15

EMERGENCY_MIN_RANGE = 1.20

CONTROL_DT = 0.05          # 20 Hz
PRESTREAM_TIME = 2.0       # send Offboard heartbeat before switching
RANGE_TIMEOUT = 0.40

MAX_TARGET_LEAD = 0.08


class SkyScrubStandoffController(Node):

    def __init__(self):

        super().__init__('skyscrub_v03_standoff_controller')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # -------------------------
        # Publishers to PX4
        # -------------------------

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            px4_qos,
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_qos,
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_qos,
        )

        # -------------------------
        # Subscriptions
        # -------------------------

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

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.status_callback,
            px4_qos,
        )

        # -------------------------
        # State
        # -------------------------

        self.range_m = None
        self.range_time = None

        self.position = None
        self.status = None

        self.initialized = False

        self.prestream_start = None
        self.last_mode_request = 0.0
        self.mode_requests = 0

        self.ever_offboard = False
        self.manual_override_detected = False

        self.forward_n = 0.0
        self.forward_e = 0.0

        self.left_n = 0.0
        self.left_e = 0.0

        self.cross_hold = 0.0
        self.z_hold = 0.0
        self.yaw_hold = 0.0

        self.cmd_s = 0.0

        self.in_standoff = False

        self.last_log = 0.0

        self.timer = self.create_timer(
            CONTROL_DT,
            self.control_loop,
        )

        self.get_logger().info(
            'SkyScrub V0.3 standoff controller started'
        )

        self.get_logger().info(
            f'Target facade range: {DESIRED_RANGE:.2f} m'
        )

        self.get_logger().info(
            f'Max approach speed: {MAX_APPROACH_SPEED:.2f} m/s'
        )

        self.get_logger().info(
            'Waiting for valid airborne PX4 state...'
        )

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------

    def range_callback(self, msg):

        valid = [
            r for r in msg.ranges
            if math.isfinite(r)
            and msg.range_min <= r <= msg.range_max
        ]

        if valid:

            self.range_m = statistics.median(valid)

            self.range_time = time.monotonic()

    def position_callback(self, msg):

        self.position = msg

    def status_callback(self, msg):

        self.status = msg

    # --------------------------------------------------
    # PX4 publishing
    # --------------------------------------------------

    def timestamp_us(self):

        return self.get_clock().now().nanoseconds // 1000

    def publish_offboard_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.timestamp_us()

        # Position controller active.
        # Velocity in TrajectorySetpoint is feed-forward.
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        self.offboard_pub.publish(msg)

    def publish_setpoint(self, speed):

        pos = self.position

        # Actual position projected along facade approach direction
        actual_s = (
            pos.x * self.forward_n +
            pos.y * self.forward_e
        )

        lead = self.cmd_s - actual_s

        # Prevent the position reference from getting too far ahead
        if lead > MAX_TARGET_LEAD:
            self.cmd_s = actual_s + MAX_TARGET_LEAD

        elif lead < -MAX_TARGET_LEAD:
            self.cmd_s = actual_s - MAX_TARGET_LEAD

        target_x = (
            self.forward_n * self.cmd_s +
            self.left_n * self.cross_hold
        )

        target_y = (
            self.forward_e * self.cmd_s +
            self.left_e * self.cross_hold
        )

        vel_n = speed * self.forward_n
        vel_e = speed * self.forward_e

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp_us()

        msg.position = [
            float(target_x),
            float(target_y),
            float(self.z_hold),
        ]

        msg.velocity = [
            float(vel_n),
            float(vel_e),
            0.0,
        ]

        nan = float('nan')

        msg.acceleration = [nan, nan, nan]
        msg.jerk = [nan, nan, nan]

        msg.yaw = float(self.yaw_hold)
        msg.yawspeed = nan

        self.trajectory_pub.publish(msg)

    def request_offboard(self):

        msg = VehicleCommand()

        msg.timestamp = self.timestamp_us()

        # PX4 custom mode -> Offboard
        msg.param1 = 1.0
        msg.param2 = 6.0

        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)

        self.mode_requests += 1

        self.get_logger().info(
            f'Requested PX4 Offboard mode '
            f'(attempt {self.mode_requests})'
        )

    # --------------------------------------------------
    # Initialization
    # --------------------------------------------------

    def initialize_reference(self):

        pos = self.position

        self.yaw_hold = pos.heading
        self.z_hold = pos.z

        # PX4 NED forward direction from heading
        self.forward_n = math.cos(self.yaw_hold)
        self.forward_e = math.sin(self.yaw_hold)

        # Horizontal left vector
        self.left_n = -self.forward_e
        self.left_e = self.forward_n

        self.cmd_s = (
            pos.x * self.forward_n +
            pos.y * self.forward_e
        )

        self.cross_hold = (
            pos.x * self.left_n +
            pos.y * self.left_e
        )

        self.initialized = True

        self.prestream_start = time.monotonic()

        self.get_logger().info(
            'Reference locked:'
        )

        self.get_logger().info(
            f'  altitude z = {self.z_hold:.3f} m NED'
        )

        self.get_logger().info(
            f'  yaw        = {self.yaw_hold:.3f} rad'
        )

        self.get_logger().info(
            f'  forward    = '
            f'[{self.forward_n:+.3f}, '
            f'{self.forward_e:+.3f}]'
        )

        self.get_logger().info(
            'Streaming hold setpoints before Offboard switch...'
        )

    # --------------------------------------------------
    # Main controller
    # --------------------------------------------------

    def control_loop(self):

        now = time.monotonic()

        if self.position is None or self.status is None:
            return

        pos = self.position
        status = self.status

        # -------------------------
        # Safety checks
        # -------------------------

        if status.failsafe:

            if now - self.last_log > 1.0:
                self.get_logger().error(
                    'PX4 FAILSAFE ACTIVE — controller not publishing'
                )
                self.last_log = now

            return

        if status.arming_state != VehicleStatus.ARMING_STATE_ARMED:

            if now - self.last_log > 1.0:
                self.get_logger().warn(
                    'Vehicle is not armed — controller idle'
                )
                self.last_log = now

            return

        if not (
            pos.xy_valid
            and pos.z_valid
            and pos.v_xy_valid
            and pos.v_z_valid
            and pos.heading_good_for_control
        ):

            if now - self.last_log > 1.0:
                self.get_logger().warn(
                    'PX4 position/heading not ready'
                )
                self.last_log = now

            return

        if self.range_m is None:

            if now - self.last_log > 1.0:
                self.get_logger().warn(
                    'Waiting for facade range sensor'
                )
                self.last_log = now

            return

        # If Offboard was manually exited, stop sending external setpoints.
        if (
            self.ever_offboard
            and status.nav_state
            != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):

            if not self.manual_override_detected:

                self.get_logger().warn(
                    'PX4 left Offboard mode. '
                    'Controller has stopped publishing setpoints.'
                )

                self.manual_override_detected = True

            return

        # -------------------------
        # Lock initial flight state
        # -------------------------

        if not self.initialized:

            self.initialize_reference()

        # -------------------------
        # Before Offboard
        # -------------------------

        if status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:

            # Hold current reference while establishing proof-of-life.
            self.publish_offboard_mode()
            self.publish_setpoint(0.0)

            elapsed = now - self.prestream_start

            if elapsed >= PRESTREAM_TIME:

                # Retry at most once per second.
                if (
                    now - self.last_mode_request >= 1.0
                    and self.mode_requests < 5
                ):

                    self.request_offboard()

                    self.last_mode_request = now

            return

        # -------------------------
        # Offboard active
        # -------------------------

        if not self.ever_offboard:

            self.ever_offboard = True

            # Start from current position to avoid setpoint jump.
            self.cmd_s = (
                pos.x * self.forward_n +
                pos.y * self.forward_e
            )

            self.get_logger().info(
                'OFFBOARD ACTIVE — beginning slow facade approach'
            )

        # Always send heartbeat in Offboard.
        self.publish_offboard_mode()

        # -------------------------
        # Range sensor freshness
        # -------------------------

        if (
            self.range_time is None
            or now - self.range_time > RANGE_TIMEOUT
        ):

            self.publish_setpoint(0.0)

            if now - self.last_log > 1.0:

                self.get_logger().warn(
                    'Range sensor stale — HOLDING position'
                )

                self.last_log = now

            return

        # -------------------------
        # Standoff controller
        # -------------------------

        error = self.range_m - DESIRED_RANGE

        # Emergency retreat if unexpectedly close
        if self.range_m < EMERGENCY_MIN_RANGE:

            speed = -MAX_RETREAT_SPEED

            self.in_standoff = False

        elif abs(error) <= DEADBAND:

            speed = 0.0

            if not self.in_standoff:

                # Lock the exact achieved position
                self.cmd_s = (
                    pos.x * self.forward_n +
                    pos.y * self.forward_e
                )

                self.in_standoff = True

                self.get_logger().info(
                    'STANDOFF ACQUIRED'
                )

        else:

            self.in_standoff = False

            speed = KP_RANGE * error

            speed = max(
                -MAX_RETREAT_SPEED,
                min(MAX_APPROACH_SPEED, speed),
            )

        # Move reference trajectory at limited speed
        self.cmd_s += speed * CONTROL_DT

        self.publish_setpoint(speed)

        # -------------------------
        # Status log
        # -------------------------

        if now - self.last_log >= 1.0:

            self.get_logger().info(
                f'range={self.range_m:.3f} m | '
                f'error={error:+.3f} m | '
                f'cmd_speed={speed:+.3f} m/s | '
                f'z={pos.z:.3f} m'
            )

            self.last_log = now


def main(args=None):

    rclpy.init(args=args)

    node = SkyScrubStandoffController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

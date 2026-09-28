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


# ----------------------------------------------------------
# FACADE STANDOFF
# ----------------------------------------------------------

DESIRED_RANGE = 2.00
RANGE_DEADBAND = 0.08

KP_RANGE = 0.25

MAX_APPROACH_SPEED = 0.20
MAX_RETREAT_SPEED = 0.15

EMERGENCY_MIN_RANGE = 1.20


# ----------------------------------------------------------
# LATERAL SCAN TEST
# ----------------------------------------------------------

LATERAL_DISTANCE = 0.80

MAX_LATERAL_SPEED = 0.12
KP_LATERAL = 0.40

LATERAL_DEADBAND = 0.05

STANDOFF_STABLE_TIME = 2.0
SIDE_HOLD_TIME = 2.0


# ----------------------------------------------------------
# CONTROLLER
# ----------------------------------------------------------

CONTROL_DT = 0.05          # 20 Hz
PRESTREAM_TIME = 2.0

RANGE_TIMEOUT = 0.40

MAX_FORWARD_LEAD = 0.08
MAX_LATERAL_LEAD = 0.08


class SkyScrubFacadeScanController(Node):

    def __init__(self):

        super().__init__('skyscrub_v03_facade_scan_controller')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --------------------------------------------------
        # PX4 publishers
        # --------------------------------------------------

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

        # --------------------------------------------------
        # Subscriptions
        # --------------------------------------------------

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

        # --------------------------------------------------
        # Sensor / PX4 state
        # --------------------------------------------------

        self.range_m = None
        self.range_time = None

        self.position = None
        self.status = None

        # --------------------------------------------------
        # Reference frame
        # --------------------------------------------------

        self.initialized = False

        self.forward_n = 0.0
        self.forward_e = 0.0

        self.left_n = 0.0
        self.left_e = 0.0

        self.z_hold = 0.0
        self.yaw_hold = 0.0

        # Coordinates in the forward / lateral basis

        self.cmd_s = 0.0
        self.cmd_c = 0.0

        self.center_c = 0.0
        self.scan_goal_c = 0.0

        # --------------------------------------------------
        # Offboard state
        # --------------------------------------------------

        self.prestream_start = None
        self.last_mode_request = 0.0
        self.mode_requests = 0

        self.ever_offboard = False
        self.manual_override_detected = False

        # --------------------------------------------------
        # Scan state machine
        # --------------------------------------------------

        self.scan_state = 'APPROACH'

        self.standoff_since = None
        self.side_hold_since = None

        self.last_log = 0.0

        self.timer = self.create_timer(
            CONTROL_DT,
            self.control_loop,
        )

        self.get_logger().info(
            'SkyScrub V0.3 facade scan controller started'
        )

        self.get_logger().info(
            f'Standoff target : {DESIRED_RANGE:.2f} m'
        )

        self.get_logger().info(
            f'Lateral test    : {LATERAL_DISTANCE:.2f} m'
        )

        self.get_logger().info(
            f'Lateral max     : {MAX_LATERAL_SPEED:.2f} m/s'
        )

        self.get_logger().info(
            'Waiting for valid airborne PX4 state...'
        )

    # ======================================================
    # CALLBACKS
    # ======================================================

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

    # ======================================================
    # HELPERS
    # ======================================================

    def timestamp_us(self):

        return self.get_clock().now().nanoseconds // 1000

    def clamp(self, value, minimum, maximum):

        return max(minimum, min(maximum, value))

    def actual_forward_coordinate(self):

        return (
            self.position.x * self.forward_n
            + self.position.y * self.forward_e
        )

    def actual_lateral_coordinate(self):

        return (
            self.position.x * self.left_n
            + self.position.y * self.left_e
        )

    def actual_lateral_velocity(self):

        return (
            self.position.vx * self.left_n
            + self.position.vy * self.left_e
        )

    # ======================================================
    # PX4 PUBLISHING
    # ======================================================

    def publish_offboard_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.timestamp_us()

        msg.position = True
        msg.velocity = False

        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        msg.thrust_and_torque = False
        msg.direct_actuator = False

        self.offboard_pub.publish(msg)

    def publish_setpoint(
        self,
        forward_speed,
        lateral_speed,
    ):

        actual_s = self.actual_forward_coordinate()
        actual_c = self.actual_lateral_coordinate()

        # --------------------------------------------------
        # Limit how far the virtual position target may move
        # ahead of the real vehicle.
        # --------------------------------------------------

        forward_lead = self.cmd_s - actual_s

        if forward_lead > MAX_FORWARD_LEAD:
            self.cmd_s = actual_s + MAX_FORWARD_LEAD

        elif forward_lead < -MAX_FORWARD_LEAD:
            self.cmd_s = actual_s - MAX_FORWARD_LEAD

        lateral_lead = self.cmd_c - actual_c

        if lateral_lead > MAX_LATERAL_LEAD:
            self.cmd_c = actual_c + MAX_LATERAL_LEAD

        elif lateral_lead < -MAX_LATERAL_LEAD:
            self.cmd_c = actual_c - MAX_LATERAL_LEAD

        # --------------------------------------------------
        # Convert facade-relative coordinates into PX4 NED
        # --------------------------------------------------

        target_x = (
            self.forward_n * self.cmd_s
            + self.left_n * self.cmd_c
        )

        target_y = (
            self.forward_e * self.cmd_s
            + self.left_e * self.cmd_c
        )

        velocity_n = (
            forward_speed * self.forward_n
            + lateral_speed * self.left_n
        )

        velocity_e = (
            forward_speed * self.forward_e
            + lateral_speed * self.left_e
        )

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp_us()

        msg.position = [
            float(target_x),
            float(target_y),
            float(self.z_hold),
        ]

        msg.velocity = [
            float(velocity_n),
            float(velocity_e),
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

        msg.param1 = 1.0
        msg.param2 = 6.0

        msg.command = (
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        )

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

    # ======================================================
    # INITIALIZATION
    # ======================================================

    def initialize_reference(self):

        pos = self.position

        self.z_hold = pos.z
        self.yaw_hold = pos.heading

        # Heading defines aircraft forward direction in PX4 NED.

        self.forward_n = math.cos(self.yaw_hold)
        self.forward_e = math.sin(self.yaw_hold)

        # Horizontal aircraft-left direction.

        self.left_n = -self.forward_e
        self.left_e = self.forward_n

        self.cmd_s = (
            pos.x * self.forward_n
            + pos.y * self.forward_e
        )

        self.cmd_c = (
            pos.x * self.left_n
            + pos.y * self.left_e
        )

        self.center_c = self.cmd_c

        self.scan_goal_c = (
            self.center_c + LATERAL_DISTANCE
        )

        self.initialized = True

        self.prestream_start = time.monotonic()

        self.get_logger().info(
            'Reference locked:'
        )

        self.get_logger().info(
            f'  z hold       = {self.z_hold:.3f} m'
        )

        self.get_logger().info(
            f'  yaw          = {self.yaw_hold:.3f} rad'
        )

        self.get_logger().info(
            f'  forward NED  = '
            f'[{self.forward_n:+.3f}, '
            f'{self.forward_e:+.3f}]'
        )

        self.get_logger().info(
            f'  left NED     = '
            f'[{self.left_n:+.3f}, '
            f'{self.left_e:+.3f}]'
        )

        self.get_logger().info(
            'Streaming hold setpoints before Offboard...'
        )

    # ======================================================
    # STANDOFF CONTROL
    # ======================================================

    def calculate_forward_speed(self):

        error = self.range_m - DESIRED_RANGE

        if self.range_m < EMERGENCY_MIN_RANGE:

            return -MAX_RETREAT_SPEED, error

        if abs(error) <= RANGE_DEADBAND:

            # Lock the current forward reference whenever
            # the desired standoff band is reached.

            self.cmd_s = self.actual_forward_coordinate()

            return 0.0, error

        speed = KP_RANGE * error

        speed = self.clamp(
            speed,
            -MAX_RETREAT_SPEED,
            MAX_APPROACH_SPEED,
        )

        return speed, error

    # ======================================================
    # LATERAL STATE MACHINE
    # ======================================================

    def calculate_lateral_speed(
        self,
        range_error,
        now,
    ):

        actual_c = self.actual_lateral_coordinate()
        lateral_velocity = self.actual_lateral_velocity()

        # --------------------------------------------------
        # 1. First acquire and stabilize facade standoff.
        # --------------------------------------------------

        if self.scan_state == 'APPROACH':

            if abs(range_error) <= RANGE_DEADBAND:

                if self.standoff_since is None:

                    self.standoff_since = now

                    self.get_logger().info(
                        'STANDOFF ACQUIRED — stabilizing'
                    )

                elif (
                    now - self.standoff_since
                    >= STANDOFF_STABLE_TIME
                ):

                    self.center_c = actual_c
                    self.cmd_c = actual_c

                    self.scan_goal_c = (
                        self.center_c
                        + LATERAL_DISTANCE
                    )

                    self.scan_state = 'SCAN_LEFT'

                    self.get_logger().info(
                        'STANDOFF STABLE — '
                        'starting lateral scan'
                    )

                    self.get_logger().info(
                        f'Lateral target offset: '
                        f'+{LATERAL_DISTANCE:.2f} m'
                    )

            else:

                self.standoff_since = None

            return 0.0

        # --------------------------------------------------
        # 2. Move sideways.
        # --------------------------------------------------

        if self.scan_state == 'SCAN_LEFT':

            error_c = self.scan_goal_c - actual_c

            if (
                abs(error_c) <= LATERAL_DEADBAND
                and abs(lateral_velocity) < 0.05
            ):

                self.cmd_c = actual_c

                self.scan_state = 'HOLD_LEFT'

                self.side_hold_since = now

                self.get_logger().info(
                    'LATERAL OFFSET ACQUIRED'
                )

                return 0.0

            speed = KP_LATERAL * error_c

            return self.clamp(
                speed,
                -MAX_LATERAL_SPEED,
                MAX_LATERAL_SPEED,
            )

        # --------------------------------------------------
        # 3. Hold at lateral endpoint.
        # --------------------------------------------------

        if self.scan_state == 'HOLD_LEFT':

            if (
                now - self.side_hold_since
                >= SIDE_HOLD_TIME
            ):

                self.scan_state = 'RETURN_CENTER'

                self.get_logger().info(
                    'RETURNING TO SCAN CENTER'
                )

            return 0.0

        # --------------------------------------------------
        # 4. Return to original lateral position.
        # --------------------------------------------------

        if self.scan_state == 'RETURN_CENTER':

            error_c = self.center_c - actual_c

            if (
                abs(error_c) <= LATERAL_DEADBAND
                and abs(lateral_velocity) < 0.05
            ):

                self.cmd_c = actual_c

                self.scan_state = 'COMPLETE'

                self.get_logger().info(
                    'LATERAL SCAN COMPLETE'
                )

                return 0.0

            speed = KP_LATERAL * error_c

            return self.clamp(
                speed,
                -MAX_LATERAL_SPEED,
                MAX_LATERAL_SPEED,
            )

        # --------------------------------------------------
        # 5. Complete — remain hovering.
        # --------------------------------------------------

        return 0.0

    # ======================================================
    # MAIN LOOP
    # ======================================================

    def control_loop(self):

        now = time.monotonic()

        if (
            self.position is None
            or self.status is None
        ):
            return

        pos = self.position
        status = self.status

        # --------------------------------------------------
        # Safety
        # --------------------------------------------------

        if status.failsafe:

            if now - self.last_log > 1.0:

                self.get_logger().error(
                    'PX4 FAILSAFE ACTIVE'
                )

                self.last_log = now

            return

        if (
            status.arming_state
            != VehicleStatus.ARMING_STATE_ARMED
        ):

            if now - self.last_log > 1.0:

                self.get_logger().warn(
                    'Vehicle not armed — controller idle'
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
                    'Waiting for facade range'
                )

                self.last_log = now

            return

        # --------------------------------------------------
        # Detect manual exit from Offboard
        # --------------------------------------------------

        if (
            self.ever_offboard
            and status.nav_state
            != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):

            if not self.manual_override_detected:

                self.get_logger().warn(
                    'PX4 left Offboard — '
                    'controller stopped publishing'
                )

                self.manual_override_detected = True

            return

        # --------------------------------------------------
        # Initialize
        # --------------------------------------------------

        if not self.initialized:

            self.initialize_reference()

        # --------------------------------------------------
        # Prestream before entering Offboard
        # --------------------------------------------------

        if (
            status.nav_state
            != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):

            self.publish_offboard_mode()

            self.publish_setpoint(
                0.0,
                0.0,
            )

            elapsed = (
                now - self.prestream_start
            )

            if elapsed >= PRESTREAM_TIME:

                if (
                    now - self.last_mode_request >= 1.0
                    and self.mode_requests < 5
                ):

                    self.request_offboard()

                    self.last_mode_request = now

            return

        # --------------------------------------------------
        # Offboard became active
        # --------------------------------------------------

        if not self.ever_offboard:

            self.ever_offboard = True

            self.cmd_s = (
                pos.x * self.forward_n
                + pos.y * self.forward_e
            )

            self.cmd_c = (
                pos.x * self.left_n
                + pos.y * self.left_e
            )

            self.get_logger().info(
                'OFFBOARD ACTIVE'
            )

            self.get_logger().info(
                'Beginning facade standoff acquisition'
            )

        self.publish_offboard_mode()

        # --------------------------------------------------
        # Check range freshness
        # --------------------------------------------------

        if (
            self.range_time is None
            or now - self.range_time
            > RANGE_TIMEOUT
        ):

            self.publish_setpoint(
                0.0,
                0.0,
            )

            if now - self.last_log > 1.0:

                self.get_logger().warn(
                    'Range stale — HOLDING'
                )

                self.last_log = now

            return

        # --------------------------------------------------
        # Forward facade-distance controller
        # --------------------------------------------------

        forward_speed, range_error = (
            self.calculate_forward_speed()
        )

        # --------------------------------------------------
        # Lateral scan controller
        # --------------------------------------------------

        lateral_speed = (
            self.calculate_lateral_speed(
                range_error,
                now,
            )
        )

        # --------------------------------------------------
        # Integrate smooth virtual targets
        # --------------------------------------------------

        self.cmd_s += (
            forward_speed * CONTROL_DT
        )

        self.cmd_c += (
            lateral_speed * CONTROL_DT
        )

        # --------------------------------------------------
        # Publish PX4 trajectory
        # --------------------------------------------------

        self.publish_setpoint(
            forward_speed,
            lateral_speed,
        )

        # --------------------------------------------------
        # Status output
        # --------------------------------------------------

        if now - self.last_log >= 1.0:

            lateral_offset = (
                self.actual_lateral_coordinate()
                - self.center_c
            )

            self.get_logger().info(
                f'state={self.scan_state} | '
                f'range={self.range_m:.3f} m | '
                f'range_err={range_error:+.3f} m | '
                f'lat_offset={lateral_offset:+.3f} m | '
                f'fwd={forward_speed:+.3f} m/s | '
                f'lat={lateral_speed:+.3f} m/s | '
                f'z={pos.z:.3f} m'
            )

            self.last_log = now


def main(args=None):

    rclpy.init(args=args)

    node = SkyScrubFacadeScanController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':

    main()

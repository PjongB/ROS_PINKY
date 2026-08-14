"""Control the final low-speed reverse docking maneuver."""

import math
from collections import deque
from statistics import median

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from pinky_project_1.pid_controller import PIDController


def normalize_angle(angle: float) -> float:
    """Normalize an angle to the [-pi, pi] range."""
    return math.atan2(math.sin(angle), math.cos(angle))


class DockingController(Node):
    """Reverse toward a rear obstacle using distance and angle PID."""

    def __init__(self) -> None:
        super().__init__('docking_controller')

        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('target_distance', 0.10)
        self.declare_parameter('distance_tolerance', 0.01)
        self.declare_parameter('docking_scan_angle_deg', 0.0)
        self.declare_parameter('rear_scan_window_deg', 8.0)
        self.declare_parameter('alignment_fit_window_deg', 25.0)
        self.declare_parameter('alignment_range_tolerance', 0.05)
        self.declare_parameter('alignment_min_points', 10)
        self.declare_parameter('angle_filter_window', 5)
        self.declare_parameter('alignment_distance', 0.10)
        self.declare_parameter('required_alignment_cycles', 5)
        self.declare_parameter('alignment_min_duration', 2.0)
        self.declare_parameter('sensor_timeout', 0.5)
        self.declare_parameter('required_stable_cycles', 5)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('linear_kp', 0.4)
        self.declare_parameter('linear_ki', 0.0)
        self.declare_parameter('linear_kd', 0.05)
        self.declare_parameter('angular_kp', 0.5)
        self.declare_parameter('angular_ki', 0.0)
        self.declare_parameter('angular_kd', 0.0)
        self.declare_parameter('max_linear_speed', 0.08)
        self.declare_parameter('max_angular_speed', 0.15)
        self.declare_parameter('angle_tolerance', 0.087)
        self.declare_parameter('docking_timeout', 45.0)

        self.control_rate = float(self.get_parameter('control_rate').value)
        self.target_distance = float(
            self.get_parameter('target_distance').value
        )
        self.distance_tolerance = float(
            self.get_parameter('distance_tolerance').value
        )
        self.docking_scan_angle = math.radians(float(
            self.get_parameter('docking_scan_angle_deg').value
        ))
        self.rear_scan_window = math.radians(float(
            self.get_parameter('rear_scan_window_deg').value
        ))
        self.alignment_fit_window = math.radians(float(
            self.get_parameter('alignment_fit_window_deg').value
        ))
        self.alignment_range_tolerance = float(
            self.get_parameter('alignment_range_tolerance').value
        )
        self.alignment_min_points = int(
            self.get_parameter('alignment_min_points').value
        )
        self.angle_tolerance = float(
            self.get_parameter('angle_tolerance').value
        )
        self.angle_filter_window = max(
            1,
            int(self.get_parameter('angle_filter_window').value),
        )
        self.alignment_distance = float(
            self.get_parameter('alignment_distance').value
        )
        self.required_alignment_cycles = int(
            self.get_parameter('required_alignment_cycles').value
        )
        self.alignment_min_duration = float(
            self.get_parameter('alignment_min_duration').value
        )
        self.sensor_timeout = float(
            self.get_parameter('sensor_timeout').value
        )
        self.required_stable_cycles = int(
            self.get_parameter('required_stable_cycles').value
        )
        self.docking_timeout = float(
            self.get_parameter('docking_timeout').value
        )
        max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.linear_pid = PIDController(
            kp=float(self.get_parameter('linear_kp').value),
            ki=float(self.get_parameter('linear_ki').value),
            kd=float(self.get_parameter('linear_kd').value),
            output_min=0.0,
            output_max=max_linear_speed,
        )
        max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self.angular_pid = PIDController(
            kp=float(self.get_parameter('angular_kp').value),
            ki=float(self.get_parameter('angular_ki').value),
            kd=float(self.get_parameter('angular_kd').value),
            output_min=-max_angular_speed,
            output_max=max_angular_speed,
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist, cmd_vel_topic, 10
        )
        self.status_publisher = self.create_publisher(
            String, '/docking/status', 10
        )
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.create_subscription(
            Bool, '/docking/start', self.start_callback, 10
        )

        self.active = False
        self.rear_distance = None
        self.raw_angle_error = None
        self.angle_error = None
        self.alignment_point_count = 0
        self.angle_error_history = deque(
            maxlen=self.angle_filter_window
        )
        self.last_scan_time = None
        self.start_time = None
        self.previous_control_time = None
        self.stable_cycles = 0
        self.alignment_cycles = 0
        self.docking_phase = 'IDLE'
        self.docking_phase_start_time = None

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop,
        )
        self.publish_status('IDLE')
        self.get_logger().info(
            'Docking controller is ready. '
            'Publish true to /docking/start to begin.'
        )

    def now_seconds(self) -> float:
        """Return the node clock as floating-point seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def publish_status(self, status: str) -> None:
        """Publish and log a docking state."""
        self.status_publisher.publish(String(data=status))
        self.get_logger().info(f'Docking status: {status}')

    def stop_robot(self) -> None:
        """Publish a zero velocity command."""
        self.cmd_vel_publisher.publish(Twist())

    def scan_callback(self, msg: LaserScan) -> None:
        """Extract rear distance and wall angle from LaserScan."""
        rear_ranges = []
        alignment_samples = []
        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            # Pinky's rplidar_link is rotated pi radians from base_link.
            # LaserScan angle 0 therefore points toward the robot rear.
            rear_error = normalize_angle(angle - self.docking_scan_angle)
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            distance = float(distance)
            if abs(rear_error) <= self.rear_scan_window:
                rear_ranges.append(distance)
            if abs(rear_error) <= self.alignment_fit_window:
                alignment_samples.append((rear_error, distance))

        self.rear_distance = median(rear_ranges) if rear_ranges else None
        self.raw_angle_error = None
        self.angle_error = None
        self.alignment_point_count = 0
        if alignment_samples:
            surface_distance = median(
                distance for _, distance in alignment_samples
            )
            points = [
                (
                    distance * math.cos(angle),
                    distance * math.sin(angle),
                )
                for angle, distance in alignment_samples
                if abs(distance - surface_distance)
                <= self.alignment_range_tolerance
            ]
            self.alignment_point_count = len(points)
        else:
            points = []

        if len(points) >= self.alignment_min_points:
            mean_x = sum(point[0] for point in points) / len(points)
            mean_y = sum(point[1] for point in points) / len(points)
            covariance_xx = sum(
                (point[0] - mean_x) ** 2 for point in points
            )
            covariance_xy = sum(
                (point[0] - mean_x) * (point[1] - mean_y)
                for point in points
            )
            covariance_yy = sum(
                (point[1] - mean_y) ** 2 for point in points
            )
            wall_angle = 0.5 * math.atan2(
                2.0 * covariance_xy,
                covariance_xx - covariance_yy,
            )
            self.raw_angle_error = normalize_angle(
                math.pi / 2.0 - wall_angle
            )
            if self.raw_angle_error > math.pi / 2.0:
                self.raw_angle_error -= math.pi
            elif self.raw_angle_error < -math.pi / 2.0:
                self.raw_angle_error += math.pi
            self.angle_error_history.append(self.raw_angle_error)
            self.angle_error = median(self.angle_error_history)
        self.last_scan_time = self.now_seconds()
        if not self.active and self.rear_distance is not None:
            self.get_logger().info(
                f'Rear target preview: {self.rear_distance:.3f} m',
                throttle_duration_sec=2.0,
            )

    def start_callback(self, msg: Bool) -> None:
        """Start docking on true and cancel docking on false."""
        if not msg.data:
            if self.active:
                self.finish_docking('CANCELED')
            return

        if self.active:
            self.get_logger().warning('Docking is already active.')
            return

        self.active = True
        self.start_time = self.now_seconds()
        self.previous_control_time = self.start_time
        self.stable_cycles = 0
        self.alignment_cycles = 0
        self.docking_phase = 'APPROACH'
        self.docking_phase_start_time = self.start_time
        self.raw_angle_error = None
        self.angle_error = None
        self.angle_error_history.clear()
        self.linear_pid.reset()
        self.angular_pid.reset()
        self.publish_status('DOCKING')

    def finish_docking(self, status: str) -> None:
        """Stop motion and finish the current docking attempt."""
        self.active = False
        self.docking_phase = 'IDLE'
        self.stop_robot()
        self.publish_status(status)

    def control_loop(self) -> None:
        """Run the rear-distance PID control loop."""
        if not self.active:
            return

        now = self.now_seconds()
        if now - self.start_time > self.docking_timeout:
            self.finish_docking('FAILED_TIMEOUT')
            return

        if (
            self.last_scan_time is None
            or now - self.last_scan_time > self.sensor_timeout
            or self.rear_distance is None
        ):
            self.stop_robot()
            self.get_logger().warning(
                'No valid rear LaserScan target; robot remains stopped.',
                throttle_duration_sec=2.0,
            )
            return

        needs_angle = self.docking_phase in ('APPROACH', 'ALIGN_IN_PLACE')
        if needs_angle and self.angle_error is None:
            self.stop_robot()
            self.get_logger().warning(
                'No valid LaserScan angle target; robot remains stopped.',
                throttle_duration_sec=2.0,
            )
            return

        distance_error = self.rear_distance - self.target_distance
        if (
            self.docking_phase == 'FINAL_APPROACH'
            and distance_error <= self.distance_tolerance
        ):
            self.stop_robot()
            self.stable_cycles += 1
            if self.stable_cycles >= self.required_stable_cycles:
                self.finish_docking('COMPLETE')
            return

        self.stable_cycles = 0
        dt = max(
            now - self.previous_control_time,
            1.0 / self.control_rate,
        )
        self.previous_control_time = now
        command = Twist()

        if self.docking_phase == 'APPROACH':
            if self.rear_distance <= self.alignment_distance:
                self.docking_phase = 'ALIGN_IN_PLACE'
                self.docking_phase_start_time = now
                self.alignment_cycles = 0
                self.angle_error_history.clear()
                self.angle_error = None
                self.angular_pid.reset()
                self.stop_robot()
                self.get_logger().info(
                    'Docking phase: ALIGN_IN_PLACE'
                )
                return

            reverse_speed = self.linear_pid.update(distance_error, dt)
            angular_speed = self.angular_pid.update(self.angle_error, dt)
            command.linear.x = -reverse_speed
            # A positive measured error means the right rear ray is farther.
            # Rotate clockwise while reversing to equalize the rear ranges.
            command.angular.z = -angular_speed

        elif self.docking_phase == 'ALIGN_IN_PLACE':
            filter_is_ready = (
                len(self.angle_error_history)
                == self.angle_error_history.maxlen
            )
            filtered_is_aligned = (
                abs(self.angle_error) <= self.angle_tolerance
            )
            minimum_time_elapsed = (
                now - self.docking_phase_start_time
                >= self.alignment_min_duration
            )
            if filter_is_ready and filtered_is_aligned and minimum_time_elapsed:
                self.alignment_cycles += 1
                self.stop_robot()
                self.get_logger().info(
                    'Alignment stable: '
                    f'{self.alignment_cycles}/'
                    f'{self.required_alignment_cycles}, '
                    f'raw={math.degrees(self.raw_angle_error):.2f} deg, '
                    f'filtered={math.degrees(self.angle_error):.2f} deg',
                    throttle_duration_sec=0.5,
                )
                if self.alignment_cycles >= self.required_alignment_cycles:
                    self.docking_phase = 'FINAL_APPROACH'
                    self.docking_phase_start_time = now
                    self.stable_cycles = 0
                    self.linear_pid.reset()
                    self.stop_robot()
                    self.get_logger().info(
                        'Docking phase: FINAL_APPROACH'
                    )
                return
            else:
                self.alignment_cycles = 0
            angular_speed = self.angular_pid.update(self.angle_error, dt)
            command.angular.z = -angular_speed

        elif self.docking_phase == 'FINAL_APPROACH':
            reverse_speed = self.linear_pid.update(distance_error, dt)
            command.linear.x = -reverse_speed

        self.cmd_vel_publisher.publish(command)
        raw_angle_text = (
            'n/a' if self.raw_angle_error is None
            else f'{math.degrees(self.raw_angle_error):.2f}'
        )
        angle_text = (
            'n/a' if self.angle_error is None
            else f'{math.degrees(self.angle_error):.2f}'
        )
        self.get_logger().info(
            f'rear={self.rear_distance:.3f} m, '
            f'error={distance_error:.3f} m, '
            f'raw_angle={raw_angle_text} deg, '
            f'angle_error={angle_text} deg, '
            f'fit_points={self.alignment_point_count}, '
            f'mode={self.docking_phase}, '
            f'cmd_vel.x={command.linear.x:.3f} m/s, '
            f'cmd_vel.z={command.angular.z:.3f} rad/s',
            throttle_duration_sec=1.0,
        )

    def destroy_node(self):
        """Stop the robot before destroying the node."""
        self.stop_robot()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DockingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

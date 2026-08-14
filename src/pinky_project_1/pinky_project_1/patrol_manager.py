"""Run the automatic start preparation, patrol, return, and docking mission."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String


class PatrolManager(Node):
    """Manage point-to-point Nav2 travel and precision docking."""

    def __init__(self) -> None:
        super().__init__('patrol_manager')

        self.declare_parameter('auto_start', True)
        self.declare_parameter('auto_patrol', True)
        self.declare_parameter('start_docked', False)
        self.declare_parameter('shutdown_on_complete', True)
        self.declare_parameter('startup_delay', 2.0)
        self.declare_parameter('departure_delay', 2.0)
        self.declare_parameter('goal_timeout', 120.0)
        self.declare_parameter('max_retries', 2)
        for name in ('a', 'b', 'c', 'd', 'e'):
            self.declare_parameter(f'point_{name}', [0.0, 0.0, 0.0])

        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.auto_patrol = bool(self.get_parameter('auto_patrol').value)
        self.start_docked = bool(
            self.get_parameter('start_docked').value
        )
        self.shutdown_on_complete = bool(
            self.get_parameter('shutdown_on_complete').value
        )
        self.startup_delay = float(
            self.get_parameter('startup_delay').value
        )
        self.departure_delay = float(
            self.get_parameter('departure_delay').value
        )
        self.goal_timeout = float(
            self.get_parameter('goal_timeout').value
        )
        self.max_retries = int(self.get_parameter('max_retries').value)
        self.points = {
            name.upper(): list(
                self.get_parameter(f'point_{name}').value
            )
            for name in ('a', 'b', 'c', 'd', 'e')
        }
        self.patrol_route = ['B', 'C', 'D', 'E', 'A']

        self.navigator = ActionClient(
            self, NavigateToPose, '/navigate_to_pose'
        )
        self.docking_start_publisher = self.create_publisher(
            Bool, '/docking/start', 10
        )
        self.status_publisher = self.create_publisher(
            String, '/project/status', 10
        )
        self.create_subscription(
            String, '/docking/status', self.docking_status_callback, 10
        )

        self.state = 'WAITING_NAV' if self.auto_start else 'IDLE'
        self.phase = 'PATROL' if self.start_docked else 'INITIAL_POSITION'
        self.current_point = 'B' if self.start_docked else 'A'
        self.route_index = 0
        self.retry_count = 0
        self.goal_handle = None
        self.goal_start_time = None
        self.ready_time = self.now_seconds() + self.startup_delay
        self.timer = self.create_timer(0.5, self.control_loop)

        self.publish_status(self.state)
        if self.auto_start:
            if self.start_docked:
                self.get_logger().info(
                    'Initial start position is ready. Starting docked at A. '
                    'Mission: B -> C -> D -> E -> A -> final docking.'
                )
            else:
                self.get_logger().info(
                    'Mission enabled: initial A docking -> B -> C -> D -> E '
                    '-> A -> final docking.'
                )
        else:
            self.get_logger().info('Automatic mission disabled.')

    def now_seconds(self) -> float:
        """Return the node clock as floating-point seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def publish_status(self, status: str) -> None:
        """Publish the project state for monitoring."""
        self.status_publisher.publish(String(data=status))

    def set_state(self, state: str) -> None:
        """Update and publish the current project state."""
        self.state = state
        self.publish_status(state)

    def control_loop(self) -> None:
        """Send pending navigation goals and monitor their timeout."""
        if self.state == 'WAITING_NAV':
            if self.now_seconds() < self.ready_time:
                return
            if not self.navigator.server_is_ready():
                self.get_logger().info(
                    'Waiting for Nav2 navigate_to_pose action server...',
                    throttle_duration_sec=3.0,
                )
                return
            self.send_navigation_goal()
            return

        if (
            self.state == 'NAVIGATING'
            and self.goal_start_time is not None
            and self.now_seconds() - self.goal_start_time > self.goal_timeout
        ):
            self.set_state('CANCELING')
            self.get_logger().error(
                f'Navigation to point {self.current_point} timed out.'
            )
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.retry_or_fail()

    def send_navigation_goal(self) -> None:
        """Send the current named point as a NavigateToPose goal."""
        x, y, yaw_deg = (
            float(value) for value in self.points[self.current_point]
        )
        yaw = math.radians(yaw_deg)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.set_state('SENDING_GOAL')
        self.get_logger().info(
            f'Moving to point {self.current_point}: '
            f'({x:.2f}, {y:.2f}, {yaw_deg:.1f} deg)'
        )
        future = self.navigator.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        """Handle Nav2 accepting or rejecting a goal."""
        try:
            self.goal_handle = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Failed to send goal: {error}')
            self.retry_or_fail()
            return

        if not self.goal_handle.accepted:
            self.get_logger().error(
                f'Nav2 rejected point {self.current_point}.'
            )
            self.retry_or_fail()
            return

        self.set_state('NAVIGATING')
        self.goal_start_time = self.now_seconds()
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future) -> None:
        """Advance the mission when the current navigation succeeds."""
        if self.state not in ('NAVIGATING', 'SENDING_GOAL'):
            return

        try:
            status = future.result().status
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Navigation result error: {error}')
            self.retry_or_fail()
            return

        # action_msgs/GoalStatus: STATUS_SUCCEEDED == 4
        if status != 4:
            self.get_logger().error(
                f'Point {self.current_point} failed (status={status}).'
            )
            self.retry_or_fail()
            return

        self.retry_count = 0
        self.goal_handle = None
        self.goal_start_time = None
        self.get_logger().info(f'Point {self.current_point} reached.')

        if self.phase == 'INITIAL_POSITION':
            self.start_docking('initial')
            return

        if self.current_point == 'A':
            self.start_docking('final')
            return

        self.route_index += 1
        self.current_point = self.patrol_route[self.route_index]
        self.set_state('WAITING_NAV')
        self.ready_time = self.now_seconds() + self.departure_delay

    def start_docking(self, label: str) -> None:
        """Start the initial or final precision docking operation."""
        self.set_state(
            'INITIAL_DOCKING' if label == 'initial' else 'FINAL_DOCKING'
        )
        self.get_logger().info(
            f'Starting {label} precision reverse docking.'
        )
        self.docking_start_publisher.publish(Bool(data=True))

    def retry_or_fail(self) -> None:
        """Retry the current point within the configured limit."""
        self.goal_handle = None
        self.goal_start_time = None
        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.set_state('WAITING_NAV')
            self.ready_time = self.now_seconds() + 1.0
            self.get_logger().warning(
                f'Retrying point {self.current_point} '
                f'({self.retry_count}/{self.max_retries}).'
            )
            return

        self.set_state('FAILED')
        self.get_logger().error(
            f'Mission failed at point {self.current_point}.'
        )

    def docking_status_callback(self, msg: String) -> None:
        """Advance after initial docking or finish after final docking."""
        if self.state not in ('INITIAL_DOCKING', 'FINAL_DOCKING'):
            return

        if msg.data == 'COMPLETE':
            if self.state == 'FINAL_DOCKING':
                self.set_state('MISSION_COMPLETE')
                self.get_logger().info(
                    'Patrol and final docking complete. '
                    'State: MISSION_COMPLETE'
                )
                if self.shutdown_on_complete:
                    self.get_logger().info(
                        'Mission complete. Shutting down pj_project.'
                    )
                    rclpy.shutdown()
                return

            self.set_state('READY')
            self.get_logger().info(
                'Initial start position is ready. State: READY'
            )
            if not self.auto_patrol:
                return

            self.phase = 'PATROL'
            self.route_index = 0
            self.current_point = self.patrol_route[self.route_index]
            self.set_state('WAITING_NAV')
            self.ready_time = self.now_seconds() + self.departure_delay
            self.get_logger().info('Starting patrol route: B -> C -> D -> E -> A')
            return

        if msg.data.startswith('FAILED') or msg.data == 'CANCELED':
            self.set_state('FAILED')
            self.get_logger().error(
                f'Docking failed. State: {msg.data}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

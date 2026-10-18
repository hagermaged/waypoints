import math
import time
from collections import deque

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from gps_interfaces.action import NavigateToGPS

# ---------------- Tuning ----------------
GOAL_TOLERANCE = 1.0     # m, goal counts as reached inside this radius
LINEAR_SPEED = 0.5       # m/s
ANGULAR_SPEED = 0.3      # rad/s
HEADING_TOL = 0.1        # rad, turn in place when the heading error is bigger
LOOP_PERIOD = 0.2        # s

GPS_TIMEOUT = 2.0        # s without a fresh GPS fix -> 'stale_gps' (keep > a few GPS periods)
STUCK_TIMEOUT = 10.0     # s of driving without progress -> 'stuck_no_progress'
MIN_PROGRESS = 0.3       # m, how much closer counts as progress
TURN_TIMEOUT = 15.0      # s of continuous turning -> 'oscillating_heading'
OSC_WINDOW = 10.0        # s window for counting turn-direction flips
OSC_FLIPS = 6            # flips inside the window -> 'oscillating_heading'


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class Waypoint(LifecycleNode):

    def __init__(self):
        super().__init__('waypoint')
        self.active = False
        self.fix = None          # (lat, lon, receive_time), replaced atomically
        self.yaw = None          # ENU yaw from odometry (0 = east, counter-clockwise)
        self._goal_seq = 0

    # ---------------- Lifecycle ----------------
    def on_configure(self, state):
        self.get_logger().info('Configuring WaypointLifecycle...')
        # Reentrant group: GPS/odom callbacks must keep running while the
        # (blocking) action callback is executing.
        cb_group = ReentrantCallbackGroup()

        self.cmd_pub = self.create_lifecycle_publisher(Twist, '/cmd_vel', 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps', self.gps_callback, 10, callback_group=cb_group)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10, callback_group=cb_group)
        self._action_server = ActionServer(
            self, NavigateToGPS, 'navigate_to_gps', self.execute_callback,
            cancel_callback=self.cancel_callback, callback_group=cb_group)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating WaypointLifecycle...')
        try:
            self.cmd_pub.on_activate(state)
        except Exception as e:
            self.get_logger().error(f'on_activate failed: {e}')
            return TransitionCallbackReturn.FAILURE
        self.active = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating WaypointLifecycle...')
        self.active = False
        self.cmd_pub.publish(Twist())
        self.cmd_pub.on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self._action_server.destroy()
        self.destroy_subscription(self.gps_sub)
        self.destroy_subscription(self.odom_sub)
        self.destroy_publisher(self.cmd_pub)
        self.fix = None
        self.yaw = None
        return TransitionCallbackReturn.SUCCESS

    # ---------------- Sensor callbacks ----------------
    def gps_callback(self, msg):
        if not self.active:
            return
        if msg.status.status < 0:  # STATUS_NO_FIX
            return
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        self.fix = (msg.latitude, msg.longitude, time.monotonic())

    def odom_callback(self, msg):
        if not self.active:
            return
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    # ---------------- Action server ----------------
    def cancel_callback(self, goal_handle):
        # Without this, rclpy rejects every cancel request by default.
        return CancelResponse.ACCEPT

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _end(self, goal_handle, result, outcome, reason):
        """Stop the rover, close the goal, and tell the caller why."""
        self._stop()
        result.success = (outcome == 'succeed')
        result.reason = reason
        if outcome == 'succeed':
            goal_handle.succeed()
        elif outcome == 'canceled':
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return result

    def execute_callback(self, goal_handle):
        result = NavigateToGPS.Result()

        if not self.active:
            self.get_logger().warn('Node not active, cannot navigate')
            return self._end(goal_handle, result, 'abort', 'node_not_active')

        goal_lat = goal_handle.request.latitude
        goal_lon = goal_handle.request.longitude
        if (not (math.isfinite(goal_lat) and math.isfinite(goal_lon))
                or abs(goal_lat) > 90.0 or abs(goal_lon) > 180.0):
            self.get_logger().error('Invalid goal coordinates')
            return self._end(goal_handle, result, 'abort', 'invalid_goal')

        # A newer goal preempts an older one that is still looping.
        self._goal_seq += 1
        my_id = self._goal_seq

        self.get_logger().info(f'Starting navigation to lat={goal_lat}, lon={goal_lon}')
        feedback_msg = NavigateToGPS.Feedback()

        start = time.monotonic()
        t_prev = start
        best_distance = None
        no_progress_s = 0.0      # driving time without getting closer
        turning_s = 0.0          # continuous turning time
        flips = deque()          # times the turn direction flipped
        last_sign = 0

        while rclpy.ok():
            now = time.monotonic()
            dt = now - t_prev
            t_prev = now

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                return self._end(goal_handle, result, 'canceled', 'canceled')
            if not self.active or my_id != self._goal_seq:
                return self._end(goal_handle, result, 'abort', 'interrupted')

            fix = self.fix
            last_fix_time = fix[2] if fix is not None else start
            if now - last_fix_time > GPS_TIMEOUT:
                self.get_logger().warn('GPS is stale')
                return self._end(goal_handle, result, 'abort', 'stale_gps')

            yaw = self.yaw
            if fix is None or yaw is None:
                if yaw is None and now - start > GPS_TIMEOUT:
                    return self._end(goal_handle, result, 'abort', 'no_odometry')
                self._stop()
                time.sleep(LOOP_PERIOD)
                continue

            lat, lon, _ = fix
            x, y = self.gps_to_xy(lat, lon, goal_lat, goal_lon)
            distance = math.hypot(x, y)

            feedback_msg.distance_remaining = distance
            feedback_msg.current_latitude = lat
            feedback_msg.current_longitude = lon
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Distance remaining: {distance:.2f} m',
                                   throttle_duration_sec=2.0)

            if distance <= GOAL_TOLERANCE:
                self.get_logger().info('Goal reached!')
                return self._end(goal_handle, result, 'succeed', 'goal_reached')

            target_angle = math.atan2(y, x)        # ENU angle: x = east, y = north
            angle_error = wrap_angle(target_angle - yaw)

            cmd = Twist()
            if abs(angle_error) > HEADING_TOL:
                cmd.angular.z = ANGULAR_SPEED if angle_error > 0 else -ANGULAR_SPEED
                turning_s += dt

                sign = 1 if angle_error > 0 else -1
                if last_sign and sign != last_sign:
                    flips.append(now)
                last_sign = sign
            else:
                cmd.linear.x = LINEAR_SPEED
                turning_s = 0.0
                if best_distance is None or distance < best_distance - MIN_PROGRESS:
                    best_distance = distance
                    no_progress_s = 0.0
                else:
                    no_progress_s += dt

            while flips and now - flips[0] > OSC_WINDOW:
                flips.popleft()

            if no_progress_s > STUCK_TIMEOUT:
                self.get_logger().warn('No progress while driving')
                return self._end(goal_handle, result, 'abort', 'stuck_no_progress')
            if turning_s > TURN_TIMEOUT or len(flips) >= OSC_FLIPS:
                self.get_logger().warn('Heading is not converging')
                return self._end(goal_handle, result, 'abort', 'oscillating_heading')

            self.cmd_pub.publish(cmd)
            time.sleep(LOOP_PERIOD)

        return self._end(goal_handle, result, 'abort', 'shutdown')

    # ---------------- GPS to XY (x = east, y = north, meters) ----------------
    def gps_to_xy(self, lat1, lon1, lat2, lon2):
        R = 6371e3
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi, dlambda = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        y_angle = math.sin(dlambda) * math.cos(phi2)
        x_angle = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
        theta = math.atan2(y_angle, x_angle)
        return distance * math.sin(theta), distance * math.cos(theta)


def main(args=None):
    rclpy.init(args=args)
    node = Waypoint()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
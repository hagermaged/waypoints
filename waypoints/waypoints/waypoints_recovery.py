import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from gps_interfaces.action import RecoveryGPS

# ---------------- Tuning ----------------
GPS_FRESH = 2.0            # s, a fix newer than this counts as "GPS is back"
WAIT_GPS_TIMEOUT = 10.0    # s, give up waiting for GPS after this
BACKUP_DURATION = 2.0      # s
BACKUP_SPEED = -0.3        # m/s
SPIN_SPEED = 0.4           # rad/s
SPIN_TOL = 0.05            # rad
SPIN_TIMEOUT = 12.0        # s (a 180 degree turn at 0.4 rad/s already takes ~7.9 s)
SPIN_SENSOR_WAIT = 5.0     # s to wait for GPS + odometry before spinning blind
BLIND_SPIN_DURATION = 3.0  # s


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class WaypointRecovery(LifecycleNode):

    def __init__(self):
        super().__init__('waypoint_recovery')
        self.active = False
        self.fix = None          # (lat, lon, receive_time), replaced atomically
        self.current_yaw = None  # ENU yaw from odometry (0 = east, counter-clockwise)
        self.target_lat = None
        self.target_lon = None
        self._goal_seq = 0

    # ---------------- Lifecycle ----------------
    def on_configure(self, state):
        self.get_logger().info('Configuring WaypointRecovery Lifecycle...')
        cb_group = ReentrantCallbackGroup()

        self.cmd_pub = self.create_lifecycle_publisher(Twist, '/cmd_vel', 10)
        self._action_server = ActionServer(
            self, RecoveryGPS, 'recover', self.execute_callback,
            cancel_callback=self.cancel_callback, callback_group=cb_group)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps', self.gps_callback, 10, callback_group=cb_group)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10, callback_group=cb_group)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating WaypointRecovery Lifecycle...')
        try:
            self.cmd_pub.on_activate(state)
        except Exception as e:
            self.get_logger().error(f'on_activate failed: {e}')
            return TransitionCallbackReturn.FAILURE
        self.active = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating WaypointRecovery Lifecycle...')
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
        self.current_yaw = None
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
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _gps_fresh(self):
        fix = self.fix
        return fix is not None and time.monotonic() - fix[2] < GPS_FRESH

    # ---------------- Action server ----------------
    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _interrupted(self, goal_handle, my_id):
        return (not self.active
                or goal_handle.is_cancel_requested
                or my_id != self._goal_seq)

    def _feedback(self, goal_handle, fb, text):
        # Only publish if the action actually has this feedback field, so a
        # differently named field in the .action file can never crash a recovery.
        if hasattr(fb, 'feedback_status'):
            fb.feedback_status = text
            goal_handle.publish_feedback(fb)

    def _end(self, goal_handle, result, outcome, status):
        self.cmd_pub.publish(Twist())
        result.success = (outcome == 'succeed')
        result.reason = status
        if outcome == 'succeed':
            goal_handle.succeed()
        elif outcome == 'canceled':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def execute_callback(self, goal_handle):
        result = RecoveryGPS.Result()

        if not self.active:
            self.get_logger().warn('Node not active, cannot recover')
            return self._end(goal_handle, result, 'abort', 'node_not_active')

        self.target_lat = goal_handle.request.target_lat
        self.target_lon = goal_handle.request.target_lon
        reason = goal_handle.request.reason
        self.get_logger().info(f'Recovery requested, reason={reason}')

        self._goal_seq += 1
        my_id = self._goal_seq
        fb = RecoveryGPS.Feedback()

        if reason == 'stale_gps':
            ok = self.do_wait_for_gps(goal_handle, my_id, fb)
        elif reason == 'stuck_no_progress':
            ok = self.do_backup(goal_handle, my_id, fb, BACKUP_DURATION, BACKUP_SPEED)
        elif reason == 'oscillating_heading':
            ok = self.do_spin(goal_handle, my_id, fb)
        else:
            self.get_logger().warn(f'Unknown recovery reason: {reason}')
            return self._end(goal_handle, result, 'abort', 'unknown_reason')

        if goal_handle.is_cancel_requested:
            return self._end(goal_handle, result, 'canceled', 'canceled')
        if ok:
            return self._end(goal_handle, result, 'succeed', f'recovery_{reason}_completed')
        return self._end(goal_handle, result, 'abort', f'recovery_{reason}_failed')

    # ---------------- Recovery behaviors ----------------
    def do_wait_for_gps(self, goal_handle, my_id, fb):
        """Succeeds only if a fresh GPS fix actually arrives."""
        self.get_logger().info('Waiting for GPS to come back...')
        start = time.monotonic()
        while time.monotonic() - start < WAIT_GPS_TIMEOUT:
            if self._interrupted(goal_handle, my_id):
                return False
            if self._gps_fresh():
                return True
            self._feedback(goal_handle, fb,
                           f'waiting for GPS {time.monotonic() - start:.1f}/{WAIT_GPS_TIMEOUT}s')
            time.sleep(0.2)
        self.get_logger().warn('GPS did not come back')
        return False

    def do_backup(self, goal_handle, my_id, fb, duration, speed):
        self.get_logger().info(f'Backing up for {duration}s...')
        cmd = Twist()
        cmd.linear.x = speed
        start = time.monotonic()
        while time.monotonic() - start < duration:
            if self._interrupted(goal_handle, my_id):
                return False
            self.cmd_pub.publish(cmd)
            self._feedback(goal_handle, fb,
                           f'backing up {time.monotonic() - start:.1f}/{duration}s')
            time.sleep(0.2)
        return True

    def do_spin(self, goal_handle, my_id, fb):
        # Wait (briefly) for a fresh GPS fix and a yaw so we can spin toward the target.
        self.get_logger().info('Waiting for GPS + odometry to compute target yaw...')
        start = time.monotonic()
        while not (self._gps_fresh() and self.current_yaw is not None):
            if self._interrupted(goal_handle, my_id):
                return False
            if time.monotonic() - start > SPIN_SENSOR_WAIT:
                self.get_logger().warn('Sensors unavailable, falling back to blind spin')
                return self.do_blind_spin(goal_handle, my_id, fb)
            time.sleep(0.2)

        lat, lon, _ = self.fix
        bearing = self.calculate_bearing(lat, lon, self.target_lat, self.target_lon)
        # bearing is compass-style (0 = north, clockwise); odometry yaw is ENU
        # (0 = east, counter-clockwise), so convert before comparing.
        target_yaw = wrap_angle(math.pi / 2.0 - bearing)
        self.get_logger().info(f'Spinning toward target_yaw={target_yaw:.2f}...')

        cmd = Twist()
        start = time.monotonic()
        while True:
            if self._interrupted(goal_handle, my_id):
                return False

            yaw_error = wrap_angle(target_yaw - self.current_yaw)
            if abs(yaw_error) < SPIN_TOL:
                self.cmd_pub.publish(Twist())
                self.get_logger().info('Target yaw reached.')
                return True
            if time.monotonic() - start > SPIN_TIMEOUT:
                self.cmd_pub.publish(Twist())
                self.get_logger().warn('Spin timed out before reaching target yaw.')
                return False

            cmd.angular.z = SPIN_SPEED if yaw_error > 0 else -SPIN_SPEED
            self.cmd_pub.publish(cmd)
            self._feedback(goal_handle, fb, f'yaw_error={yaw_error:.2f}')
            time.sleep(0.1)

    def do_blind_spin(self, goal_handle, my_id, fb):
        self.get_logger().info(f'Blind spin for {BLIND_SPIN_DURATION}s (no GPS/odometry)...')
        cmd = Twist()
        cmd.angular.z = SPIN_SPEED
        start = time.monotonic()
        while time.monotonic() - start < BLIND_SPIN_DURATION:
            if self._interrupted(goal_handle, my_id):
                return False
            self.cmd_pub.publish(cmd)
            self._feedback(goal_handle, fb,
                           f'blind spinning {time.monotonic() - start:.1f}/{BLIND_SPIN_DURATION}s')
            time.sleep(0.2)
        return True

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlambda = math.radians(lon2 - lon1)
        y = math.sin(dlambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
        return math.atan2(y, x)  # compass bearing: 0 = north, clockwise


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecovery()
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
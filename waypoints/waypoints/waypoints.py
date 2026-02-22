import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from gps_interfaces.action import NavigateToGPS
import math


class Waypoint(Node):

    def __init__(self):
        super().__init__('waypoint_client')

        self.lat = None
        self.long = None
        self.current_yaw = 0.0

        # Read goal coordinates from file
        self.goal_lat, self.goal_long = self.read_waypoint_from_file('waypoint.txt')
        self.get_logger().info(f'Goal loaded: lat={self.goal_lat}, lon={self.goal_long}')

        # Publisher for moving the robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to GPS
        self.create_subscription(
            NavSatFix,
            '/gps',
            self.gps_callback,
            10
        )

        # Create the action client
        self._action_client = ActionClient(self, NavigateToGPS, 'navigate_to_gps')

        # Send the goal to the server
        self.send_goal()

    # ---------------- File reading ----------------
    def read_waypoint_from_file(self, filename):
        try:
            with open(filename, 'r') as f:
                lat = float(f.readline().strip())
                lon = float(f.readline().strip())
            return lat, lon
        except FileNotFoundError:
            self.get_logger().error(f'File {filename} not found!')
            return None, None
        except ValueError:
            self.get_logger().error(f'Invalid format in {filename}')
            return None, None

    # ---------------- GPS callback ----------------
    def gps_callback(self, msg):
        if self.goal_lat is None or self.goal_long is None:
            self.get_logger().warn('Goal coordinates not loaded, skipping calculation')
            return

        self.lat = msg.latitude
        self.long = msg.longitude
        self.get_logger().info(f'Current GPS: lat={self.lat}, lon={self.long}')

        x, y = self.gps_to_xy(self.lat, self.long, self.goal_lat, self.goal_long)
        self.get_logger().info(f'Relative position to goal: x={x:.2f} m, y={y:.2f} m')

        #  Move rover toward target
        distance = math.sqrt(x*x + y*y)
        target_angle = math.atan2(y, x)
        angle_error = target_angle - self.current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        cmd = Twist()
        if distance > 1.0:   # stop tolerance (1 meter)
            if abs(angle_error) > 0.1:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.3 if angle_error > 0 else -0.3
            else:
                cmd.linear.x = 0.5
                cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("Goal reached!")

        self.cmd_pub.publish(cmd)

    # ---------------- Action client ----------------
    def send_goal(self):
        if self.goal_lat is None or self.goal_long is None:
            self.get_logger().warn('No goal loaded, cannot send')
            return

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateToGPS.Goal()
        goal_msg.latitude = self.goal_lat
        goal_msg.longitude = self.goal_long

        self.get_logger().info(f'Sending goal to server: lat={goal_msg.latitude}, lon={goal_msg.longitude}')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted by server')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {distance:.2f} m')

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().warn('Navigation failed')

    # ---------------- Calculating function ----------------
    def gps_to_xy(self, lat1, lon1, lat2, lon2):
        R = 6371e3
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = (math.sin(dphi/2) ** 2 +
             math.cos(phi1) * math.cos(phi2) *
             math.sin(dlambda/2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        y_angle = math.sin(dlambda) * math.cos(phi2)
        x_angle = (math.cos(phi1) * math.sin(phi2) -
                   math.sin(phi1) * math.cos(phi2) * math.cos(dlambda))
        theta = math.atan2(y_angle, x_angle)

        dx = distance * math.sin(theta)
        dy = distance * math.cos(theta)

        return dx, dy


def main(args=None):
    rclpy.init(args=args)
    node = Waypoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

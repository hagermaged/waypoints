import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from gps_interfaces.action import NavigateToGPS
import math


class Waypoint(Node):

    def __init__(self):
        super().__init__('waypoint')

        self.lat = None
        self.long = None
        self.current_yaw = 0.0

        # Read goal from file
        self.goal_lat, self.goal_long = self.read_waypoint_from_file('waypoint.txt')
        self.get_logger().info(f'Goal loaded: lat={self.goal_lat}, lon={self.goal_long}')

        # Publisher for moving the robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to GPS
        self.create_subscription(NavSatFix, '/gps', self.gps_callback, 10)

        # Create the action server
        self._action_server = ActionServer(self, NavigateToGPS,'navigate_to_gps',self.execute_callback)

        self.get_logger().info('Action server is up and waiting for commands...')

    # ---------------- File reading ----------------
    def read_waypoint_from_file(self, filename):
        try:
            with open(filename, 'r') as f:
                lat = float(f.readline().strip())
                lon = float(f.readline().strip())
            return lat, lon
        except (FileNotFoundError, ValueError):
            self.get_logger().error(f'Error reading {filename}')
            return None, None

    # ---------------- GPS callback ----------------
    def gps_callback(self, msg):
        self.lat = msg.latitude
        self.long = msg.longitude

    # ---------------- Action server ----------------
    def execute_callback(self, goal_handle):
        command = goal_handle.request.command
        self.get_logger().info(f'Received command: {command}')

        if command.lower() != 'go':
            self.get_logger().warn('Unknown command')
            goal_handle.abort()
            result = NavigateToGPS.Result()
            result.success = False
            return result

        if self.goal_lat is None or self.goal_long is None:
            self.get_logger().error('No waypoint loaded')
            goal_handle.abort()
            result = NavigateToGPS.Result()
            result.success = False
            return result

        self.get_logger().info(f'Starting navigation to lat={self.goal_lat}, lon={self.goal_long}')

        feedback_msg = NavigateToGPS.Feedback()
        result = NavigateToGPS.Result()

        # Timer for periodic navigation
        def navigate_step():
            if self.lat is None or self.long is None:
                self.get_logger().info('Waiting for GPS fix...')
                return

            # Calculate distance and angle
            x, y = self.gps_to_xy(self.lat, self.long, self.goal_lat, self.goal_long)
            distance = math.sqrt(x*x + y*y)
            target_angle = math.atan2(y, x)
            angle_error = math.atan2(math.sin(target_angle - self.current_yaw),
                                     math.cos(target_angle - self.current_yaw))

            # Send feedback
            feedback_msg.distance_remaining = distance
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Distance remaining: {distance:.2f} m')

            # Move robot
            cmd = Twist()
            if distance > 1.0:
                if abs(angle_error) > 0.1:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.3 if angle_error > 0 else -0.3
                else:
                    cmd.linear.x = 0.5
                    cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info('Goal reached!')
                goal_handle.succeed()
                result.success = True
                self.nav_timer.cancel()  # stop timer

            self.cmd_pub.publish(cmd)

        # Start timer to check GPS and move every 0.5 sec
        self.nav_timer = self.create_timer(0.5, navigate_step)

        # Keep action alive until it succeeds
        while not goal_handle.is_cancel_requested and not result.success:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not result.success:
            goal_handle.abort()
            result.success = False

        return result

    # ---------------- GPS to XY ----------------
    def gps_to_xy(self, lat1, lon1, lat2, lon2):
        R = 6371e3
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi, dlambda = math.radians(lat2-lat1), math.radians(lon2-lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
        c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R*c
        y_angle = math.sin(dlambda)*math.cos(phi2)
        x_angle = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlambda)
        theta = math.atan2(y_angle, x_angle)
        return distance*math.sin(theta), distance*math.cos(theta)


def main(args=None):
    rclpy.init(args=args)
    node = Waypoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
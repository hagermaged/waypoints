#!/usr/bin/env python3
"""Fake rover: listens to /cmd_vel, publishes /gps and /odometry/filtered.

Fault injection (change live with `ros2 param set /fake_rover <name> <value>`):
  stuck        true  -> wheels spin, rover does not move  (-> stuck_no_progress)
  gps_enabled  false -> stop publishing /gps              (-> stale_gps)
  freeze_yaw   true  -> odometry yaw stops updating       (-> oscillating_heading)
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus

R_EARTH = 6371e3


class FakeRover(Node):
    def __init__(self):
        super().__init__('fake_rover')
        self.declare_parameter('start_lat', 31.2000)
        self.declare_parameter('start_lon', 29.9187)
        self.declare_parameter('start_yaw_deg', 0.0)   # 0 = east, CCW positive (ENU)
        self.declare_parameter('stuck', False)
        self.declare_parameter('gps_enabled', True)
        self.declare_parameter('freeze_yaw', False)

        self.lat0 = self.get_parameter('start_lat').value
        self.lon0 = self.get_parameter('start_lon').value
        self.yaw = math.radians(self.get_parameter('start_yaw_deg').value)
        self.reported_yaw = self.yaw
        self.x = 0.0   # east, m
        self.y = 0.0   # north, m
        self.v = 0.0
        self.w = 0.0
        self.last_cmd = self.get_clock().now()

        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.dt = 0.05
        self.create_timer(self.dt, self.step)
        self.create_timer(0.2, self.pub_gps)
        self.create_timer(2.0, self.log_pose)

    def cmd_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last_cmd = self.get_clock().now()

    def step(self):
        # deadman: no command for 0.5 s -> stop (like a real motor controller)
        if (self.get_clock().now() - self.last_cmd).nanoseconds > 0.5e9:
            self.v = self.w = 0.0
        stuck = self.get_parameter('stuck').value
        v = 0.0 if stuck else self.v
        w = 0.0 if stuck else self.w
        self.yaw = math.atan2(math.sin(self.yaw + w * self.dt),
                              math.cos(self.yaw + w * self.dt))
        self.x += v * math.cos(self.yaw) * self.dt
        self.y += v * math.sin(self.yaw) * self.dt
        if not self.get_parameter('freeze_yaw').value:
            self.reported_yaw = self.yaw

        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = 'odom'
        od.pose.pose.orientation.z = math.sin(self.reported_yaw / 2.0)
        od.pose.pose.orientation.w = math.cos(self.reported_yaw / 2.0)
        self.odom_pub.publish(od)

    def lat_lon(self):
        lat = self.lat0 + math.degrees(self.y / R_EARTH)
        lon = self.lon0 + math.degrees(self.x / (R_EARTH * math.cos(math.radians(self.lat0))))
        return lat, lon

    def pub_gps(self):
        if not self.get_parameter('gps_enabled').value:
            return
        lat, lon = self.lat_lon()
        m = NavSatFix()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'gps'
        m.status.status = NavSatStatus.STATUS_FIX
        m.latitude, m.longitude = lat, lon
        self.gps_pub.publish(m)

    def log_pose(self):
        lat, lon = self.lat_lon()
        self.get_logger().info(
            f'pos lat={lat:.6f} lon={lon:.6f} yaw={math.degrees(self.yaw):.0f}deg '
            f'(stuck={self.get_parameter("stuck").value}, '
            f'gps={self.get_parameter("gps_enabled").value}, '
            f'freeze_yaw={self.get_parameter("freeze_yaw").value})')


def main():
    rclpy.init()
    node = FakeRover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

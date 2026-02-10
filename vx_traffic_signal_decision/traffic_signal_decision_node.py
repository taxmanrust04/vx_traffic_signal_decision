#!/usr/bin/env python3
 
import rclpy                         # ROS2 python library
from rclpy.node import Node          # base class for ROS node
from rclpy.time import Time          # time helper
from nav_msgs.msg import Odometry    # robot position message
from std_msgs.msg import Bool        # simple true/false message
from vx_custom_msgs.msg import TrafficSignalState  # traffic signal message
import math                          # math functions
 
 
class TrafficLightDecision(Node):
 
    def __init__(self):
        super().__init__('vx_traffic_signal_decision')   # name of node
 
        # store robot state
        self.current_position = None
        self.current_heading = None
 
        # store active intersection info
        self.active_intersection_id = None
        self.active_signal_group_id = None
 
        # remember last signals
        self.signal_cache = {}
 
        # how long signal is valid
        self.SIGNAL_TIMEOUT_SEC = 2.0
 
        # areas for each signal group
        self.signal_group_boundaries = {
            1: {
                1: (4.4, 2.0, 5.0, 3.0, (135, 225)),
                2: (3.0, 1.0, 4.0, 1.6, (45, 135)),
                3: (2.0, 2.0, 2.6, 3.0, (315, 45)),
                4: (3.0, 3.0, 4.0, 3.6, (225, 315)),
            },
            3: {
                5: (1.4, 2.0, 1.9, 3.0, (135, 225)),
                6: (0.0, 1.0, 1.0, 1.6, (45, 135)),
                8: (0.0, 3.0, 1.0, 3.6, (225, 315)),
            }
        }
 
        # publisher for stop command
        self.traffic_signal_pub = self.create_publisher(Bool, '/stop_signal', 10)
 
        # subscribe to robot pose
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
 
        # subscribe to traffic signals
        self.create_subscription(TrafficSignalState, '/traffic_signal_state', self.traffic_status_callback, 10)
 
        # timer to check signals
        self.create_timer(1.0, self.periodic_check)
 
        self.get_logger().info('Traffic Light Responder initialized')
 
    # get robot position
    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
 
        # convert quaternion to heading
        self.current_heading = self.quaternion_to_heading(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
 
        # update which signal is active
        self.update_active_signal_group()
 
    # convert quaternion to degrees
    @staticmethod
    def quaternion_to_heading(x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)
 
        # keep angle 0 to 360
        if yaw_deg < 0:
            yaw_deg += 360
        return yaw_deg
 
    # find which signal group robot is in
    def update_active_signal_group(self):
        if self.current_position is None or self.current_heading is None:
            self.active_intersection_id = None
            self.active_signal_group_id = None
            return
 
        x, y = self.current_position
        found = False
 
        # check all boundaries
        for intersection_id, signal_groups in self.signal_group_boundaries.items():
            for signal_group_id, bbox in signal_groups.items():
                if self.point_in_bbox(x, y, bbox) and self.heading_in_range(self.current_heading, bbox):
                    self.active_intersection_id = intersection_id
                    self.active_signal_group_id = signal_group_id
                    found = True
                    break
            if found:
                break
 
        # if none matched
        if not found:
            self.active_intersection_id = None
            self.active_signal_group_id = None
 
    # check point inside box
    @staticmethod
    def point_in_bbox(x, y, bbox):
        x1, y1, x2, y2 = bbox[:4]
        return x1 <= x <= x2 and y1 <= y <= y2
 
    # check heading in allowed range
    @staticmethod
    def heading_in_range(heading, bbox):
        heading_min, heading_max = bbox[4]
        return ((heading - heading_min) % 360) <= ((heading_max - heading_min) % 360)
 
    # receive traffic signal message
    def traffic_status_callback(self, msg: TrafficSignalState):
        key = (msg.intersection_id, msg.signal_group_id)
 
        # ignore state 0
        if msg.event_state == 0:
            return
 
        # save signal
        self.signal_cache[key] = {
            "event_state": msg.event_state,
            "last_update": self.get_clock().now()
        }
 
        # if this signal is active
        if (msg.intersection_id == self.active_intersection_id and
            msg.signal_group_id == self.active_signal_group_id):
            self.evaluate_signal(msg.event_state)
 
    # decide stop or go
    def evaluate_signal(self, event_state):
        if event_state in [3, 4, 7]:
            self.publish_decision(True, f"RED (event_state={event_state})")
        elif event_state in [5]:
            self.publish_decision(False, f"GREEN (event_state={event_state})")
        else:
            self.publish_decision(True, f"UNKNOWN (event_state={event_state}) -> Default STOP")
 
    # publish decision
    def publish_decision(self, stop: bool, reason: str):
        msg = Bool()
        msg.data = stop
        self.traffic_signal_pub.publish(msg)
 
        decision = "STOP" if stop else "GO"
        heading_info = f" | Heading: {self.current_heading:.1f}°" if self.current_heading is not None else ""
        self.get_logger().info(f"{decision} -> {reason}{heading_info}")
 
    # check signal every second
    def periodic_check(self):
        if self.active_intersection_id and self.active_signal_group_id:
            key = (self.active_intersection_id, self.active_signal_group_id)
 
            # no signal received
            if key not in self.signal_cache:
                self.publish_decision(True, "No traffic signal information -> STOP")
                return
 
            signal = self.signal_cache[key]
            now = self.get_clock().now()
 
            # age of signal
            age = (now - signal["last_update"]).nanoseconds * 1e-9
 
            # signal too old
            if age > self.SIGNAL_TIMEOUT_SEC:
                self.publish_decision(True, "Traffic signal information stale -> STOP")
                return
 
            # evaluate latest signal
            self.evaluate_signal(signal["event_state"])
 
        else:
            # outside intersection
            self.publish_decision(False, "Outside intersection zones -> GO")
 
 
# main start function
def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDecision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
# run script
if __name__ == '__main__':
    main()
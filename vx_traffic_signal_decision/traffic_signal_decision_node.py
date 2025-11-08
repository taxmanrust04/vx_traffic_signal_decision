#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from vx_custom_msgs.msg import TrafficSignalState
import math


class TrafficLightDecision(Node):
    # ROS 2 node that reacts to traffic light states and odometry position

    def __init__(self):
        super().__init__('vx_traffic_signal_decision')

        # State variables
        self.current_position = None
        self.current_heading = None
        self.active_intersection_id = None
        self.active_signal_group_id = None
        self.signal_cache = {}

        # Intersection zones with position and heading constraints
        # Format: (x1, y1, x2, y2, (heading_min, heading_max))
        self.signal_group_boundaries = {
            1: {
                1: (4.0, 2.0, 5.5, 3.0, (135, 225)),
                2: (3.0, 1.0, 4.0, 2.0, (45, 135)),
                3: (2.0, 2.0, 3.0, 3.0, (315, 45)),
                4: (3.0, 3.0, 4.0, 4.3, (225, 315)),
            },
            2: {
                5: (1.0, 2.0, 1.8, 3.0, (135, 225)),
                6: (0.0, 1.0, 1.0, 2.0, (45, 135)),
                8: (0.0, 3.0, 1.0, 4.3, (225, 315)),
            }
        }

        # Publishers and subscribers
        self.traffic_signal_pub = self.create_publisher(Bool, '/stop', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(TrafficSignalState, '/traffic_signal_state', self.traffic_status_callback, 10)

        # Timer for periodic re-evaluation
        self.create_timer(1.0, self.periodic_check)

        self.get_logger().info('Traffic Light Responder initialized')

    # Update current vehicle position and heading
    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        self.current_heading = self.quaternion_to_heading(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        
        self.update_active_signal_group()

    # Convert quaternion to heading in degrees [0, 360)
    @staticmethod
    def quaternion_to_heading(x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        
        yaw_deg = math.degrees(yaw_rad)
        if yaw_deg < 0:
            yaw_deg += 360
            
        return yaw_deg

    # Determine which bounding box the vehicle is in based on position and heading
    def update_active_signal_group(self):
        if self.current_position is None or self.current_heading is None:
            self.active_intersection_id = None
            self.active_signal_group_id = None
            return

        x, y = self.current_position
        found = False

        for intersection_id, signal_groups in self.signal_group_boundaries.items():
            for signal_group_id, bbox in signal_groups.items():
                if self.point_in_bbox(x, y, bbox) and self.heading_in_range(self.current_heading, bbox):
                    self.active_intersection_id = intersection_id
                    self.active_signal_group_id = signal_group_id
                    found = True
                    break
            if found:
                break

        if not found:
            self.active_intersection_id = None
            self.active_signal_group_id = None

    # Check if point (x, y) is within bounding box
    @staticmethod
    def point_in_bbox(x, y, bbox):
        x1, y1, x2, y2 = bbox[:4]
        return x1 <= x <= x2 and y1 <= y <= y2

    # Check if vehicle heading is within allowed range
    @staticmethod
    def heading_in_range(heading, bbox):        
        heading_min, heading_max = bbox[4]
        return ((heading - heading_min) % 360) <= ((heading_max - heading_min) % 360)

    # Handle incoming traffic signal messages
    def traffic_status_callback(self, msg: TrafficSignalState):
        intersection_id = msg.intersection_id
        signal_group_id = msg.signal_group_id
        event_state = msg.event_state

        # Cache latest signal
        self.signal_cache[(intersection_id, signal_group_id)] = event_state

        # React only if in the same intersection zone
        if (intersection_id == self.active_intersection_id and
            signal_group_id == self.active_signal_group_id):
            self.evaluate_signal(event_state)

    # Convert ETSI event_state to stop/go decision
    def evaluate_signal(self, event_state):
        # ETSI ITS: (2-4, 7-9) = RED; (5-6) = GREEN; others = UNKNOWN -> STOP
        if event_state in [3, 4, 7]:
            self.publish_decision(True, f"RED (event_state={event_state})")
        elif event_state in [5]:
            self.publish_decision(False, f"GREEN (event_state={event_state})")
        else:
            self.publish_decision(True, f"UNKNOWN (event_state={event_state}) -> Default STOP")

    # Publish stop/go decision
    def publish_decision(self, stop: bool, reason: str):
        msg = Bool()
        msg.data = stop
        self.traffic_signal_pub.publish(msg)
        decision = "STOP" if stop else "GO"
        
        heading_info = f" | Heading: {self.current_heading:.1f}°" if self.current_heading is not None else ""
        self.get_logger().info(f"{decision} -> {reason}{heading_info}")

    # Re-evaluate cached signals periodically
    def periodic_check(self):
        if self.active_intersection_id and self.active_signal_group_id:
            key = (self.active_intersection_id, self.active_signal_group_id)
            if key in self.signal_cache:
                self.evaluate_signal(self.signal_cache[key])
        else:
            self.publish_decision(False, "Outside intersection zones -> GO")


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDecision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


import pytest
from unittest.mock import MagicMock, ANY
import rclpy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from vx_custom_msgs.msg import TrafficSignalState
from traffic_signal_decision.traffic_signal_decision_node import TrafficLightDecision
from tf_transformations import quaternion_from_euler
import math

@pytest.fixture(scope="module")
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def node(rclpy_init_shutdown):
    node = TrafficLightDecision()
    node.get_logger = MagicMock()  # Silence logs
    node.traffic_signal_pub.publish = MagicMock()
    return node

def make_odom(x, y, yaw=0.0):
    quaternion = quaternion_from_euler(0, 0, yaw)
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.x = quaternion[0]
    msg.pose.pose.orientation.y = quaternion[1]
    msg.pose.pose.orientation.z = quaternion[2]
    msg.pose.pose.orientation.w = quaternion[3]
    return msg

def make_signal_state(intersection, group, state):
    msg = TrafficSignalState()
    msg.intersection_id = intersection
    msg.signal_group_id = group
    msg.event_state = state
    return msg

# ---------------- Coordinate Logic Tests ----------------

# TC_TSD_001: Test if point is inside the bounding box
def test_point_in_bbox_inside():
    box = (1.0, 1.0, 2.0, 2.0, 0, 360)
    assert TrafficLightDecision.point_in_bbox(1.2, 1.4, box)

# TC_TSD_002: Test if point is outside the bounding box
def test_point_in_bbox_outside():
    box = (1.0, 1.0, 2.0, 2.0, 0, 360)
    assert not TrafficLightDecision.point_in_bbox(3.5, 1.5, box)

# ---------------- Odom Callback ----------------

# TC_TSD_003: Test if odom callback correctly updates position and heading
def test_odom_callback_updates_position_heading(node):
    msg = make_odom(4.5, 2.5)
    node.odom_callback(msg)
    assert node.current_position == (4.5, 2.5)
    assert node.current_heading is not None

# ---------------- Traffic Status Callback ----------------

# TC_TSD_004: Test traffic status callback when intersection and signal group match
def test_traffic_status_callback_cache_and_match(node):
    node.active_intersection_id = 1
    node.active_signal_group_id = 3
    msg = make_signal_state(1, 3, 3)
    node.evaluate_signal = MagicMock()
    node.traffic_status_callback(msg)
    assert (1, 3) in node.signal_cache
    node.evaluate_signal.assert_called_once_with(3)

# TC_TSD_005: Test traffic status callback when intersection and signal group do not match
def test_traffic_status_callback_no_match(node):
    node.active_intersection_id = 2
    node.active_signal_group_id = 5
    msg = make_signal_state(1, 3, 3)
    node.evaluate_signal = MagicMock()
    node.traffic_status_callback(msg)
    node.evaluate_signal.assert_not_called()

# ---------------- Evaluate Signal ----------------

# TC_TSD_006: Test signal evaluation logic based on event state
@pytest.mark.parametrize("event_state,expected", [
    (3, True), (4, True), (7, True), (5, False), (99, True)
])
def test_evaluate_signal(node, event_state, expected):
    node.publish_decision = MagicMock()
    node.evaluate_signal(event_state)
    node.publish_decision.assert_called_once_with(expected, ANY)

# ---------------- Publish Decision ----------------

# TC_TSD_007: Test publishing on /stop when traffic signal state changes
def test_publish_decision_publishes_bool(node):
    node.publish_decision(True, "test")
    node.traffic_signal_pub.publish.assert_called_once()
    published_msg = node.traffic_signal_pub.publish.call_args[0][0]
    assert isinstance(published_msg, Bool)
    assert published_msg.data is True

# ---------------- Periodic Check ----------------

# TC_TSD_008: Test periodic check when there is an active signal
def test_periodic_check_with_active_signal(node):
    node.active_intersection_id = 1
    node.active_signal_group_id = 3
    node.signal_cache[1, 3] = 4
    node.evaluate_signal = MagicMock()
    node.periodic_check()
    node.evaluate_signal.assert_called_once_with(4)

# TC_TSD_009: Test periodic check when there is no active signal
def test_periodic_check_no_active_signal(node):
    node.active_intersection_id = None
    node.active_signal_group_id = None
    node.publish_decision = MagicMock()
    node.periodic_check()
    node.publish_decision.assert_called_once_with(False, ANY)

# ---------------- Active Signal Group Update ----------------

# TC_TSD_010: Test active signal group update when position or heading is None
def test_update_active_signal_group_no_position_or_heading(node):
    node.current_position = None
    node.current_heading = None
    node.update_active_signal_group()
    assert node.active_intersection_id is None
    assert node.active_signal_group_id is None

# TC_TSD_011: Test conversion of quaternion to heading in degrees
def test_quaternion_to_heading_conversion(node):
    yaw = math.radians(90)
    quaternion = quaternion_from_euler(0, 0, yaw)
    msg = make_odom(4.5, 2.5, yaw=yaw)
    node.odom_callback(msg)
    expected_heading = 90.0
    assert node.current_heading == pytest.approx(expected_heading, rel=1e-9)

# TC_TSD_012: Test publishing on /stop when signal state changes
def test_signal_state_to_stop_publish(node):
    msg_red = make_signal_state(1, 3, 3)  # RED signal
    node.active_intersection_id = 1
    node.active_signal_group_id = 3
    node.publish_decision = MagicMock()
    node.traffic_status_callback(msg_red)
    node.publish_decision.assert_called_once_with(True, ANY)

    msg_green = make_signal_state(1, 3, 5)  # GREEN signal
    node.publish_decision = MagicMock()
    node.traffic_status_callback(msg_green)
    node.publish_decision.assert_called_once_with(False, ANY)



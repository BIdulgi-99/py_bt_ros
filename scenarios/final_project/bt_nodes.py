import math
import rclpy
from rclpy.node import Node as RosNode
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

# [핵심] QR 프로세서 및 Config 임포트
from modules.utils import config  
from modules.qr_processor import AsyncQRProcessor

# [수정] Fallback, ReactiveSequence 등 제어 노드 추가 Import
from modules.base_bt_nodes import (
    Node, Status, Sequence, Fallback, ReactiveSequence, ReactiveFallback, 
    BTNodeList as BaseBTNodeList
)
from modules.base_bt_nodes_ros import ActionWithROSAction

def deg(d: float) -> float:
    return math.radians(d)

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def _build_nav_goal(x: float, y: float, yaw: float, agent) -> NavigateToPose.Goal:
    goal = NavigateToPose.Goal()
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = agent.ros_bridge.node.get_clock().now().to_msg()
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = 0.0
    ps.pose.orientation = yaw_to_quaternion(yaw)
    goal.pose = ps
    return goal

# [고정 좌표 설정]
CHARGE_X,  CHARGE_Y,  CHARGE_YAW  = -4.198, 0.2, deg(89.274)
PICKUP_X,  PICKUP_Y,  PICKUP_YAW  = -6.326, 3.209, deg(-78.415)

# --- Action Nodes ---

class MoveToPickup(ActionWithROSAction):
    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, blackboard):
        return _build_nav_goal(PICKUP_X, PICKUP_Y, PICKUP_YAW, agent)

class MoveToCharge(ActionWithROSAction):
    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, blackboard):
        return _build_nav_goal(CHARGE_X, CHARGE_Y, CHARGE_YAW, agent)

class MoveToDelivery(ActionWithROSAction):
    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, blackboard):
        pose = blackboard.get("qr_target_pose", None)
        if pose is None:
            return None
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

# --- Condition/Logic Nodes ---

class WaitForQRPose(Node):
    def __init__(self, name, agent):
        super().__init__(name)
        self.agent = agent
        self.processor = AsyncQRProcessor(config)
        self._node = agent.ros_bridge.node
        
        topic_name = config['qr_system']['camera_topic']
        self._sub = self._node.create_subscription(
            Image, topic_name, self._img_callback, 10
        )
        self._node.get_logger().info(f"[WaitForQRPose] Scanning on {topic_name}")

    def _img_callback(self, msg: Image):
        self.processor.update_image(msg)

    async def run(self, agent, blackboard):
        result = self.processor.get_result()
        if result is None:
            return Status.RUNNING # 계속 대기
        
        x, y, yaw_deg = result
        ps = PoseStamped()
        ps.header.stamp = self._node.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        
        yaw_rad = math.radians(yaw_deg)
        ps.pose.orientation.z = math.sin(yaw_rad / 2.0)
        ps.pose.orientation.w = math.cos(yaw_rad / 2.0)

        blackboard["qr_target_pose"] = ps
        self._node.get_logger().info(f"[WaitForQRPose] Target Found: ({x}, {y})")
        return Status.SUCCESS

# --- Node Registration ---
class BTNodeList:
    CONTROL_NODES = BaseBTNodeList.CONTROL_NODES
    ACTION_NODES = [
        "MoveToPickup", "MoveToDelivery", "MoveToCharge",
        "WaitForQRPose"
    ]
    CONDITION_NODES = []
    DECORATOR_NODES = []
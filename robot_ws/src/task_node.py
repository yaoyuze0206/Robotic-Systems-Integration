#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Int16
from std_msgs.msg import String as StrMsg
import yaml
from ament_index_python.packages import get_package_share_directory

class TaskNode(Node):
    def __init__(self):
        super().__init__('task_node')
        pkg_dir = get_package_share_directory('pi_bot')
        with open(pkg_dir+'/config/params.yaml','r') as f:
            self.param = yaml.safe_load(f)
        with open(pkg_dir+'/config/qr_mapping.yaml','r') as f:
            self.wps = yaml.safe_load(f)['waypoints']

        self.lift_pub = self.create_publisher(Int16, '/elevator_height', 10)
        self.qr_sub   = self.create_subscription(StrMsg, '/qr_result', self.qr_cb, 10)
        self._action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.current_rack = None
        self.timer = self.create_timer(1.0, self.start_follow)  # 延迟启动

    def start_follow(self):
        self.timer.cancel()
        self.get_logger().info('等待 nav2 就绪...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 就绪，开始搬运')
        self.follow_list(['rack_a','dest','rack_b','dest',
                          'rack_c','dest','rack_d','dest'])

    def follow_list(self, name_list):
        poses = []
        for name in name_list:
            x,y,yaw = self.wps[name]
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = self.yaw_to_quat(yaw)
            poses.append(pose)
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses
        self.get_logger().info(f'发送 waypoint 列表: {name_list}')
        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result().result
        if result.result:
            self.get_logger().info('全部搬运完成！')
        else:
            self.get_logger().warn('任务失败')
        rclpy.shutdown()

    def qr_cb(self, msg: StrMsg):
        if self.current_rack and msg.data.startswith(self.current_rack.upper()):
            self.get_logger().info(f'QR 正确: {msg.data}')
            # 继续下一个 waypoint 由 Nav2 自动做

    def yaw_to_quat(self, yaw):
        from math import sin, cos
        return Quaternion(x=0.0, y=0.0, z=sin(yaw/2), w=cos(yaw/2))

def main():
    rclpy.init()
    node = TaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
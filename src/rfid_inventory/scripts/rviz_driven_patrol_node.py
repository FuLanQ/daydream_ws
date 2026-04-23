#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import SetBool
from std_msgs.msg import String

class RvizDrivenPatrolNode(Node):
    def __init__(self):
        super().__init__('rviz_driven_patrol_node')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rfid_client = self.create_client(SetBool, '/rfid/inventory_switch')
        
        self.rfid_sub = self.create_subscription(String, '/rfid/tag_data_raw', self.rfid_callback, 10)
        self.inventory_result_pub = self.create_publisher(String, '/rfid/inventory_results', 10)

        # ⭐ 新增：订阅 Rviz2 的 "2D Nav Goal" 点击事件
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.rviz_goal_callback, 10)

        self.collected_tags = set()
        self.is_collecting = False
        
        self.get_logger().info('🟢 智能巡检系统已启动，请在 Rviz2 中点击 [2D Nav Goal] 设定盘点目标...')

    # ⭐ 当你在 Rviz2 中点击鼠标时，这部分代码会被唤醒
    def rviz_goal_callback(self, rviz_msg):
        self.get_logger().info(f'📍 收到 Rviz2 的目标指令: X={rviz_msg.pose.position.x:.2f}, Y={rviz_msg.pose.position.y:.2f}')
        
        # 截获 rviz 发来的消息，将其包装成 Action 请求发送给底层导航
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = rviz_msg
        
        # 发送目标并等待到达
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('被 Nav2 拒绝执行该目标。')
            return
        
        self.get_logger().info('🚗 小车开始前往你鼠标点击的地点...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result.status == 4: # 到达目标
            self.get_logger().info('🎯 已到达 Rviz 所指地点，启动雷达旋转扫卡！')

            # 开启硬件 -> 旋转提取 -> 关闭硬件
            self.collected_tags.clear()
            self.is_collecting = True
            self.set_rfid_state(True)
            self.manual_spin(spin_seconds=13.0)
            self.set_rfid_state(False)
            self.is_collecting = False

            # 汇总查阅
            final_list = list(self.collected_tags)
            result_msg = String()
            result_msg.data = ",".join(final_list) 
            self.inventory_result_pub.publish(result_msg)
            
            self.get_logger().info(f'📢 已向全网发布 {len(final_list)} 条记录。等待你的下一次鼠标点击...')

    def rfid_callback(self, msg):
        if self.is_collecting and msg.data:
            [self.collected_tags.add(t) for t in msg.data.split(';') if t.strip()]

    def set_rfid_state(self, state: bool):
        if self.rfid_client.wait_for_service(timeout_sec=2.0):
            req = SetBool.Request()
            req.data = state
            self.rfid_client.call_async(req)

    def manual_spin(self, spin_seconds):
        self.get_logger().info('🔄 原地自转扫码中...')
        twist = Twist()
        twist.angular.z = 1.0 
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < spin_seconds:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = RvizDrivenPatrolNode()
    rclpy.spin(node) # 保持运行，全天候等待接收 Rviz 点击
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
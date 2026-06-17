import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys
import tty
import termios
import threading
import numpy as np

# 匯入原本 TidyBot 的底盤控制核心
from base_controller import Vehicle 

class TidyBotRosNode(Node):
    def __init__(self):
        super().__init__('tidybot_base_node')
        
        # 1. 初始化 TidyBot 實體底盤
        self.get_logger().info('正在初始化 TidyBot 馬達底盤...')
        self.vehicle = Vehicle() 
        
        # 🔥 【核心修正】：必須主動喚醒原作者的 4ms 背景控制迴圈，否則指令佇列會塞滿！
        self.get_logger().info('🔥 正在啟動底層高頻控制執行緒...')
        self.vehicle.start_control()
        
        # 2. 訂閱標準 ROS 2 速度 Topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 3. 安全機制變數設定
        self.is_unlocked = False          # 預設鎖定狀態
        self.last_cmd_time = time.time()  # 記錄最後收到指令的時間
        self.timeout_duration = 0.3        # 超過 0.3 秒沒收到指令就急停 (看門狗)
        
        # 4. 建立安全看門狗定時器
        self.safety_timer = self.create_timer(0.1, self.safety_check_callback)
        
        self.get_logger().info('==================================================')
        self.get_logger().info('🔴 TidyBot 目前處於【鎖定狀態】。')
        self.get_logger().info('👉 請在驅動視窗按下【空白鍵】解鎖/急停。')
        self.get_logger().info('==================================================')

        # 5. 開啟獨立執行緒監聽本機鍵盤的空白鍵
        self.keyboard_thread = threading.Thread(target=self.listen_spacebar)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def cmd_vel_callback(self, msg):
        """ 當收到速度指令時執行 """
        self.last_cmd_time = time.time()
        
        # 檢查安全解鎖狀態
        if not self.is_unlocked:
            self.vehicle.set_target_velocity(np.array([0.0, 0.0, 0.0]))
            return

        # 已解鎖狀態，正常發送速度
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        self.vehicle.set_target_velocity(np.array([vx, vy, wz]), frame='local')

    def safety_check_callback(self):
        """ 看門狗安全檢查 """
        now = time.time()
        if self.is_unlocked and (now - self.last_cmd_time) > self.timeout_duration:
            self.vehicle.set_target_velocity(np.array([0.0, 0.0, 0.0]))
            self.get_logger().warn('【安全警告】未偵測到控制訊號，強制將馬達急停！', throttle_duration_sec=2.0)

    def listen_spacebar(self):
        """ 獨立監聽空白鍵的函式 """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                ch = sys.stdin.read(1)
                if ch == ' ':  # 按下空白鍵
                    self.is_unlocked = not self.is_unlocked
                    if self.is_unlocked:
                        self.get_logger().info('🟢 【安全解鎖】TidyBot 已解鎖，可接受 ROS 2 速度指令控制。')
                    else:
                        self.vehicle.set_target_velocity(np.array([0.0, 0.0, 0.0]))
                        self.get_logger().error('🔴 【緊急停止】TidyBot 已強制煞車並鎖定！')
                elif ch == '\x03':  # Ctrl+C 退出
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = TidyBotRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('正在關閉 ROS 2 節點...')
    finally:
        # 🔥 【核心修正】：安全關閉時，除了給零速度，也要記得優雅關閉控制迴圈
        node.get_logger().info('正在安全關閉底層控制執行緒...')
        node.vehicle.set_target_velocity(np.array([0.0, 0.0, 0.0]))
        node.vehicle.stop_control()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
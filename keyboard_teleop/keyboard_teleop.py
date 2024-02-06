import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import sys, termios, tty, select, os
import threading
import yaml

class KeyTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # ターミナル設定の保存。キーボード入力のための特殊設定を後で元に戻すため
        if os.isatty(sys.stdin.fileno()):
            self.settings = termios.tcgetattr(sys.stdin)
        else:
            self.settings = None  # sys.stdinが端末デバイスに接続されていない場合

        self.pub_twist = self.create_publisher(Twist, '/cmd_vel', 10)

        # コマンドバインディングの設定
        # キー入力に対応する移動方向と回転方向のベクトルを定義
        self.cmd_bindings = {
            'q': np.array([1, 1]),
            'w': np.array([1, 0]),
            'e': np.array([1, -1]),
            'a': np.array([0, 1]),
            'd': np.array([0, -1]),
            'z': np.array([-1, -1]),
            'x': np.array([-1, 0]),
            'c': np.array([-1, 1])
        }
        # スピード設定バインディング
        # 特定のキーでスピードの増減を行う
        self.set_bindings = {
            't': np.array([1, 1]),
            'b': np.array([-1, -1]),
            'y': np.array([1, 0]),
            'n': np.array([-1, 0]),
            'u': np.array([0, 1]),
            'm': np.array([0, -1])
        }
        self.speed = np.array([0.5, 1.0]) # 初期スピード設定
        self.inc_ratio = 0.1 # スピード変更比率
        self.command = np.array([0, 0]) # 現在のコマンド（スピードと方向）
        #self.settings = termios.tcgetattr(sys.stdin) # ターミナル設定の保存。キーボード入力のための特殊設定を後で元に戻すため

        # 使用方法表示
        self.print_usage()
        
        # キーボード入力監視用のスレッドを開始
        self.input_thread = threading.Thread(target=self.monitor_keyboard_input)
        self.input_thread.daemon = True  # プログラム終了時にスレッドも終了するように設定
        self.input_thread.start()
        
        # タイマーを設定して、キー入力がなくても定期的にTwistメッセージをパブリッシュ
        self.timer = self.create_timer(0.1, self.publish_twist_message)
        self.get_logger().info('Node is running')

    def monitor_keyboard_input(self):
        """キーボード入力を監視し、コマンドを更新する"""
        while True:
            # キーボード入力を非ブロッキングで取得
            ch = self.get_key()
            if ch:
                self.process_key(ch)

    def publish_twist_message(self):
        """設定されたコマンドに基づいてTwistメッセージをパブリッシュ"""
        self.update()

    def fini(self):
        """プログラム終了時にターミナル設定を元に戻す"""
        if self.settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def print_usage(self):
        msg = """
        Keyboard Teleop that Publish to /cmd_vel (geometry_msgs/Twist)
        Copyright (C) 2024
        Released under Apache License
        --------------------------------------------------
        H:       Print this menu
        Moving around:
          Q   W   E
          A   S   D
          Z   X   C
        T/B :   increase/decrease max speeds 10%
        Y/N :   increase/decrease only linear speed 10%
        U/M :   increase/decrease only angular speed 10%
        anything else : stop

        G :   Quit
        --------------------------------------------------
        """
        self.get_logger().info(msg)
        self.show_status()

    def show_status(self):
        msg = 'Status:\tlinear {:.2f}\tangular {:.2f}'.format(self.speed[0], self.speed[1])
        self.get_logger().info(msg)

    def process_key(self, ch):
        if ch == 'h':
            self.print_usage()
        elif ch in self.cmd_bindings:
            self.command = self.cmd_bindings[ch]
        elif ch in self.set_bindings:
            self.speed = self.speed * (1 + self.set_bindings[ch] * self.inc_ratio)
            self.show_status()
        elif ch == 'g':
            self.get_logger().info('Quitting')
            self.fini()  # ターミナル設定を復元
            twist = Twist()
            self.pub_twist.publish(twist)
            rclpy.shutdown()
        else:
            self.command = np.array([0, 0])

    def update(self):
        twist = Twist()
        cmd = self.speed * self.command
        twist.linear.x = cmd[0]
        twist.angular.z = cmd[1]
        self.pub_twist.publish(twist)

    def get_key(self):
        # ターミナル設定を変更してキー入力を非ブロッキングで取得
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        return key.lower()

def main(args=None):
    rclpy.init(args=args)
    teleop = KeyTeleop()
    try:
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        pass
    finally:
        teleop.fini()  # プログラム終了時にターミナルの設定を復元
        teleop.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
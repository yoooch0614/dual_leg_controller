#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math

class JoystickStatusDisplay:
    def __init__(self):
        rospy.init_node('joystick_status_display', anonymous=True)
        
        # パラメータ読み込み
        self.display_rate = rospy.get_param('~display_rate', 5.0)
        
        # サブスクライバー
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.enable_sub = rospy.Subscriber('/hexapod/enable', Bool, self.enable_callback)
        
        # 状態変数
        self.last_joy = None
        self.last_cmd_vel = None
        self.is_enabled = False
        self.joy_connected = False
        
        # 表示タイマー
        self.display_timer = rospy.Timer(
            rospy.Duration(1.0 / self.display_rate), 
            self.display_status
        )
        
        rospy.loginfo("Joystick Status Display initialized")
        
    def joy_callback(self, msg):
        self.last_joy = msg
        self.joy_connected = True
        
    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg
        
    def enable_callback(self, msg):
        self.is_enabled = msg.data
        
    def display_status(self, event):
        # 画面クリア（デバッグモード時のみ）
        if rospy.get_param('~clear_screen', False):
            print('\033[2J\033[H', end='')
        
        print("=" * 60)
        print("    6脚ロボット ジョイスティック制御状態")
        print("=" * 60)
        
        # 接続状態
        if self.joy_connected and self.last_joy is not None:
            print(f"✓ ジョイスティック: 接続済み ({len(self.last_joy.axes)}軸, {len(self.last_joy.buttons)}ボタン)")
        else:
            print("✗ ジョイスティック: 未接続")
            
        # システム状態
        status_color = "🟢" if self.is_enabled else "🔴"
        print(f"{status_color} システム状態: {'有効' if self.is_enabled else '無効'}")
        
        # ジョイスティック入力値
        if self.last_joy is not None and len(self.last_joy.axes) >= 4:
            left_x = self.last_joy.axes[0]   # 左スティック横
            left_y = self.last_joy.axes[1]   # 左スティック縦
            right_x = self.last_joy.axes[3]  # 右スティック横
            
            # デッドゾーン考慮
            deadzone = 0.1
            left_magnitude = math.sqrt(left_x**2 + left_y**2)
            
            print("\n📋 ジョイスティック入力:")
            print(f"  左スティック: X={left_x:+.2f}, Y={left_y:+.2f} (強度: {left_magnitude:.2f})")
            print(f"  右スティック: X={right_x:+.2f}")
            
            if left_magnitude > deadzone:
                direction = math.atan2(left_y, left_x) * 180.0 / math.pi
                print(f"  方向: {direction:+.1f}° {'(デッドゾーン外)' if left_magnitude > deadzone else '(デッドゾーン内)'}")
            else:
                print("  方向: --- (デッドゾーン内)")
                
        # ボタン状態
        if self.last_joy is not None and len(self.last_joy.buttons) >= 9:
            print("\n🎮 ボタン状態:")
            button_names = {
                4: "L1/LB (有効化)",
                5: "R1/RB (緊急停止)",
                8: "Start (ホーム)"
            }
            
            for btn_idx, btn_name in button_names.items():
                if btn_idx < len(self.last_joy.buttons):
                    status = "押下中" if self.last_joy.buttons[btn_idx] else "未押下"
                    print(f"  {btn_name}: {status}")
                    
        # cmd_vel出力値
        if self.last_cmd_vel is not None:
            linear_speed = math.sqrt(self.last_cmd_vel.linear.x**2 + self.last_cmd_vel.linear.y**2)
            print(f"\n🤖 出力コマンド:")
            print(f"  直進速度: {linear_speed*1000:.1f} mm/s")
            print(f"  角速度: {self.last_cmd_vel.angular.z:.3f} rad/s")
            
            if linear_speed > 0.001:
                direction = math.atan2(self.last_cmd_vel.linear.y, self.last_cmd_vel.linear.x) * 180.0 / math.pi
                print(f"  移動方向: {direction:+.1f}°")
        else:
            print("\n🤖 出力コマンド: なし")
            
        print("\n💡 操作方法:")
        print("  L1/LB: システム有効化/無効化")
        print("  左スティック: 移動制御 (前後・左右)")  
        print("  右スティック: 回転制御")
        print("  R1/RB: 緊急停止")
        print("  Start: ホームポジション")
        print("=" * 60)
        print()

def main():
    try:
        display = JoystickStatusDisplay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nジョイスティック状態表示を終了します")
        sys.exit(0)

if __name__ == '__main__':
    main()
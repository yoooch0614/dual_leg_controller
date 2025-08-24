#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64
import math

class EnhancedJoystickStatusDisplay:
    def __init__(self):
        rospy.init_node('enhanced_joystick_status_display', anonymous=True)
        
        # パラメータ読み込み
        self.display_rate = rospy.get_param('~display_rate', 3.0)
        self.show_parameter_hints = rospy.get_param('~show_parameter_hints', True)
        
        # サブスクライバー
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.enable_sub = rospy.Subscriber('/hexapod/enable', Bool, self.enable_callback)
        
        # パラメータ監視サブスクライバー
        self.step_height_sub = rospy.Subscriber('/hexapod/param/step_height', Float64, self.step_height_callback)
        self.step_length_sub = rospy.Subscriber('/hexapod/param/step_length', Float64, self.step_length_callback)
        self.cycle_time_sub = rospy.Subscriber('/hexapod/param/cycle_time', Float64, self.cycle_time_callback)
        self.max_speed_sub = rospy.Subscriber('/hexapod/param/max_speed', Float64, self.max_speed_callback)
        
        # 状態変数
        self.last_joy = None
        self.last_cmd_vel = None
        self.is_enabled = False
        self.joy_connected = False
        self.is_moving = False
        
        # パラメータ状態
        self.current_step_height = 17.0
        self.current_step_length = 60.0
        self.current_cycle_time = 1.2
        self.current_max_speed = 80.0
        self.param_mode = 0  # 0=速度, 1=高さ, 2=長さ, 3=時間
        
        # 表示タイマー
        self.display_timer = rospy.Timer(
            rospy.Duration(1.0 / self.display_rate), 
            self.display_status
        )
        
        rospy.loginfo("Enhanced Joystick Status Display initialized")
        
    def joy_callback(self, msg):
        self.last_joy = msg
        self.joy_connected = True
        
        # パラメータ調整モード検出（Select/Backボタンの押下を検出）
        if len(msg.buttons) > 9 and msg.buttons[9]:  # Select/Back
            # モード切替のタイミングを検出するため、フラグベースで管理
            pass
        
    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg
        # 移動状態判定
        linear_speed = math.sqrt(msg.linear.x**2 + msg.linear.y**2)
        self.is_moving = (linear_speed > 0.001 or abs(msg.angular.z) > 0.001)
        
    def enable_callback(self, msg):
        self.is_enabled = msg.data
        
    def step_height_callback(self, msg):
        self.current_step_height = msg.data
        
    def step_length_callback(self, msg):
        self.current_step_length = msg.data
        
    def cycle_time_callback(self, msg):
        self.current_cycle_time = msg.data
        
    def max_speed_callback(self, msg):
        self.current_max_speed = msg.data
        
    def display_status(self, event):
        # 画面クリア（デバッグモード時のみ）
        if rospy.get_param('~clear_screen', False):
            print('\033[2J\033[H', end='')
        
        print("=" * 70)
        print("    6脚ロボット ジョイスティック制御状態（強化版）")
        print("=" * 70)
        
        # 接続状態
        if self.joy_connected and self.last_joy is not None:
            print(f"✓ ジョイスティック: 接続済み ({len(self.last_joy.axes)}軸, {len(self.last_joy.buttons)}ボタン)")
        else:
            print("✗ ジョイスティック: 未接続")
            
        # システム状態
        if self.is_enabled:
            if self.is_moving:
                status_color = "🟢"
                status_text = "有効・移動中"
            else:
                status_color = "🟡"
                status_text = "有効・待機中（静止）"
        else:
            status_color = "🔴"
            status_text = "無効・完全停止"
            
        print(f"{status_color} システム状態: {status_text}")
        
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
                print(f"  方向: {direction:+.1f}° (デッドゾーン外)")
            else:
                print("  方向: --- (デッドゾーン内・静止)")
                
        # ボタン状態（重要なもののみ表示）
        if self.last_joy is not None and len(self.last_joy.buttons) >= 10:
            print("\n🎮 ボタン状態:")
            
            important_buttons = {
                4: ("L1/LB", "有効化切替"),
                5: ("R1/RB", "緊急停止"),
                8: ("Start", "ホーム"),
                9: ("Select", "パラメータモード"),
                3: ("Y/△", "パラメータ↑"),
                0: ("A/X", "パラメータ↓")
            }
            
            for btn_idx, (btn_name, function) in important_buttons.items():
                if btn_idx < len(self.last_joy.buttons):
                    status = "🔴押下中" if self.last_joy.buttons[btn_idx] else "⚪未押下"
                    print(f"  {btn_name} ({function}): {status}")
                    
        # cmd_vel出力値
        if self.last_cmd_vel is not None:
            linear_speed = math.sqrt(self.last_cmd_vel.linear.x**2 + self.last_cmd_vel.linear.y**2)
            print(f"\n🤖 出力コマンド:")
            if linear_speed > 0.001 or abs(self.last_cmd_vel.angular.z) > 0.001:
                print(f"  直進速度: {linear_speed*1000:.1f} mm/s")
                print(f"  角速度: {self.last_cmd_vel.angular.z:.3f} rad/s")
                
                if linear_speed > 0.001:
                    direction = math.atan2(self.last_cmd_vel.linear.y, self.last_cmd_vel.linear.x) * 180.0 / math.pi
                    print(f"  移動方向: {direction:+.1f}°")
            else:
                print("  🛑 完全停止状態（足は静止）")
        else:
            print("\n🤖 出力コマンド: なし（待機中）")
            
        # 現在のパラメータ表示
        print(f"\n⚙️  現在のパラメータ:")
        param_names = ["最大速度", "ステップ高さ", "ステップ長さ", "サイクル時間"]
        param_values = [
            f"{self.current_max_speed:.1f} mm/s",
            f"{self.current_step_height:.1f} mm", 
            f"{self.current_step_length:.1f} mm",
            f"{self.current_cycle_time:.1f} s"
        ]
        
        for i, (name, value) in enumerate(zip(param_names, param_values)):
            indicator = "👈 調整中" if i == self.param_mode else ""
            print(f"  {name}: {value} {indicator}")
            
        if self.show_parameter_hints:
            print(f"\n💡 パラメータ調整:")
            print(f"  現在の調整対象: {param_names[self.param_mode]}")
            print(f"  Select/Back: 調整対象切替")
            print(f"  Y/△ボタン: 値を増加")
            print(f"  A/Xボタン: 値を減少")
            
        print("\n💡 基本操作:")
        print("  L1/LB: システム有効化/無効化")
        print("  左スティック: 移動制御 (前後・左右)")  
        print("  右スティック: 回転制御")
        print("  R1/RB: 緊急停止")
        print("  Start: ホームポジション")
        print("=" * 70)
        print()

def main():
    try:
        display = EnhancedJoystickStatusDisplay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n強化版ジョイスティック状態表示を終了します")
        sys.exit(0)

if __name__ == '__main__':
    main()
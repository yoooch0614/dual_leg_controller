#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import dynamic_reconfigure.client

class HexapodParameterAdjuster:
    def __init__(self):
        rospy.init_node('hexapod_parameter_adjuster', anonymous=True)
        
        # パラメータ読み込み
        self.param_mode_button = rospy.get_param('~param_mode_button', 9)  # Select/Back
        self.param_up_button = rospy.get_param('~param_up_button', 3)      # Y/Triangle
        self.param_down_button = rospy.get_param('~param_down_button', 0)  # A/X
        
        # パラメータ範囲設定
        self.step_height_min = rospy.get_param('~step_height_min', 5.0)
        self.step_height_max = rospy.get_param('~step_height_max', 30.0)
        self.step_height_step = rospy.get_param('~step_height_step', 2.0)
        
        self.step_length_min = rospy.get_param('~step_length_min', 20.0)
        self.step_length_max = rospy.get_param('~step_length_max', 100.0)
        self.step_length_step = rospy.get_param('~step_length_step', 5.0)
        
        self.cycle_time_min = rospy.get_param('~cycle_time_min', 0.6)
        self.cycle_time_max = rospy.get_param('~cycle_time_max', 2.5)
        self.cycle_time_step = rospy.get_param('~cycle_time_step', 0.1)
        
        self.max_speed_min = rospy.get_param('~max_speed_min', 10.0)
        self.max_speed_max = rospy.get_param('~max_speed_max', 150.0)
        self.max_speed_step = rospy.get_param('~max_speed_step', 10.0)
        
        # 現在のパラメータ値
        self.current_step_height = 17.0
        self.current_step_length = 60.0
        self.current_cycle_time = 1.2
        self.current_max_speed = 80.0
        
        # 調整モード（0=最大速度, 1=ステップ高さ, 2=ステップ長さ, 3=サイクル時間）
        self.param_mode = 0
        
        # ボタン状態記録（エッジ検出用）
        self.last_buttons = {}
        
        # サブスクライバー
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        
        # ROSパラメータ更新用（dynamic_reconfigureの代替）
        self.update_timer = rospy.Timer(rospy.Duration(0.1), self.update_ros_parameters)
        
        rospy.loginfo("Hexapod Parameter Adjuster initialized")
        self.print_current_mode()
        
    def joy_callback(self, msg):
        """ジョイスティックコールバック"""
        if len(msg.buttons) < max(self.param_mode_button, self.param_up_button, self.param_down_button) + 1:
            return
            
        # ボタンエッジ検出
        mode_pressed = self.detect_button_edge(msg, self.param_mode_button)
        up_pressed = self.detect_button_edge(msg, self.param_up_button)
        down_pressed = self.detect_button_edge(msg, self.param_down_button)
        
        # ボタン処理
        if mode_pressed:
            self.switch_parameter_mode()
            
        if up_pressed:
            self.adjust_parameter(True)
            
        if down_pressed:
            self.adjust_parameter(False)
            
        # ボタン状態を記録
        self.last_buttons = {}
        for i, button in enumerate(msg.buttons):
            self.last_buttons[i] = button
    
    def detect_button_edge(self, msg, button_index):
        """ボタンの立ち上がりエッジを検出"""
        if button_index >= len(msg.buttons):
            return False
            
        current_state = msg.buttons[button_index]
        last_state = self.last_buttons.get(button_index, False)
        
        return current_state and not last_state
    
    def switch_parameter_mode(self):
        """パラメータ調整モードの切替"""
        self.param_mode = (self.param_mode + 1) % 4
        self.print_current_mode()
    
    def adjust_parameter(self, increase):
        """パラメータの調整"""
        step_multiplier = 1.0 if increase else -1.0
        
        if self.param_mode == 0:  # 最大速度
            self.current_max_speed += step_multiplier * self.max_speed_step
            self.current_max_speed = max(self.max_speed_min, 
                                       min(self.max_speed_max, self.current_max_speed))
            rospy.loginfo(f"最大速度: {self.current_max_speed:.1f} mm/s")
            
        elif self.param_mode == 1:  # ステップ高さ
            self.current_step_height += step_multiplier * self.step_height_step
            self.current_step_height = max(self.step_height_min, 
                                         min(self.step_height_max, self.current_step_height))
            rospy.loginfo(f"ステップ高さ: {self.current_step_height:.1f} mm")
            
        elif self.param_mode == 2:  # ステップ長さ
            self.current_step_length += step_multiplier * self.step_length_step
            self.current_step_length = max(self.step_length_min, 
                                         min(self.step_length_max, self.current_step_length))
            rospy.loginfo(f"ステップ長さ: {self.current_step_length:.1f} mm")
            
        elif self.param_mode == 3:  # サイクル時間
            self.current_cycle_time += step_multiplier * self.cycle_time_step
            self.current_cycle_time = max(self.cycle_time_min, 
                                        min(self.cycle_time_max, self.current_cycle_time))
            rospy.loginfo(f"サイクル時間: {self.current_cycle_time:.1f} s")
    
    def update_ros_parameters(self, event):
        """ROSパラメータサーバーを更新"""
        try:
            # enhanced_smooth_hexapod_controllerノードのパラメータを更新
            rospy.set_param('/enhanced_smooth_hexapod_controller/step_height', self.current_step_height)
            rospy.set_param('/enhanced_smooth_hexapod_controller/step_length', self.current_step_length)
            rospy.set_param('/enhanced_smooth_hexapod_controller/cycle_time', self.current_cycle_time)
            
            # ジョイスティックコントローラーの最大速度も更新
            rospy.set_param('/hexapod_joystick_controller/max_linear_speed', self.current_max_speed)
            
        except Exception as e:
            # パラメータ更新エラーは警告程度に
            pass
    
    def print_current_mode(self):
        """現在の調整モードを表示"""
        mode_names = ["最大速度", "ステップ高さ", "ステップ長さ", "サイクル時間"]
        mode_values = [
            f"{self.current_max_speed:.1f} mm/s",
            f"{self.current_step_height:.1f} mm", 
            f"{self.current_step_length:.1f} mm",
            f"{self.current_cycle_time:.1f} s"
        ]
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("パラメータ調整モード切替")
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"現在の調整対象: {mode_names[self.param_mode]}")
        rospy.loginfo(f"現在値: {mode_values[self.param_mode]}")
        rospy.loginfo("")
        rospy.loginfo("📋 全パラメータ:")
        for i, (name, value) in enumerate(zip(mode_names, mode_values)):
            indicator = "👈 調整中" if i == self.param_mode else ""
            rospy.loginfo(f"  {name}: {value} {indicator}")
        rospy.loginfo("")
        rospy.loginfo("💡 操作方法:")
        rospy.loginfo("  Select/Back: 調整対象切替")
        rospy.loginfo("  Y/△ボタン: 値を増加")
        rospy.loginfo("  A/Xボタン: 値を減少")
        rospy.loginfo("=" * 50)

def main():
    try:
        adjuster = HexapodParameterAdjuster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("パラメータ調整ノードを終了します")

if __name__ == '__main__':
    main()
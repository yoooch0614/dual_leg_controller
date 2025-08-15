#!/usr/bin/env python3
"""
脚の安全監視ノード
異常な位置や速度を検出して警告を発する
"""

import rospy
import math
from dual_leg_controller.msg import LegPosition, LegCommand
from std_msgs.msg import String
from geometry_msgs.msg import Point

class LegSafetyMonitor:
    def __init__(self):
        rospy.init_node('leg_safety_monitor', anonymous=True)
        
        # パラメータ取得
        self.target_leg = rospy.get_param('~target_leg', 'RF')
        self.position_tolerance = rospy.get_param('~position_tolerance', 10.0)
        self.velocity_limit = rospy.get_param('~velocity_limit', 100.0)
        
        # 安全範囲（パラメータから取得）
        self.safety_limits = {
            'x_min': rospy.get_param('/safety/x_limit_min', 80.0),
            'x_max': rospy.get_param('/safety/x_limit_max', 200.0),
            'y_min': rospy.get_param('/safety/y_limit_min', -50.0),
            'y_max': rospy.get_param('/safety/y_limit_max', 50.0),
            'z_min': rospy.get_param('/safety/z_limit_min', -120.0),
            'z_max': rospy.get_param('/safety/z_limit_max', -50.0),
        }
        
        # 状態変数
        self.last_position = None
        self.last_time = None
        self.warning_count = 0
        self.max_warnings = 3
        
        # Subscriber
        self.cmd_sub = rospy.Subscriber(
            f'/asterisk/leg/{self.target_leg}/command/foot_position',
            LegPosition, self.command_callback)
        
        self.state_sub = rospy.Subscriber(
            f'/asterisk/leg/{self.target_leg}/state/foot_position',
            LegPosition, self.state_callback)
        
        # Publisher
        self.warning_pub = rospy.Publisher('/safety_warnings', String, queue_size=10)
        self.emergency_pub = rospy.Publisher('/emergency_stop', String, queue_size=1)
        
        rospy.loginfo(f"Safety monitor started for leg {self.target_leg}")
        rospy.loginfo(f"Safety limits: x[{self.safety_limits['x_min']:.1f}, {self.safety_limits['x_max']:.1f}], "
                     f"y[{self.safety_limits['y_min']:.1f}, {self.safety_limits['y_max']:.1f}], "
                     f"z[{self.safety_limits['z_min']:.1f}, {self.safety_limits['z_max']:.1f}]")
    
    def command_callback(self, msg):
        """コマンド位置の安全性チェック"""
        if not self.is_position_safe(msg.x, msg.y, msg.z):
            warning_msg = f"UNSAFE COMMAND for {self.target_leg}: [{msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}]"
            rospy.logwarn(warning_msg)
            self.warning_pub.publish(String(data=warning_msg))
            
            self.warning_count += 1
            if self.warning_count >= self.max_warnings:
                self.publish_emergency_stop("Too many unsafe commands")
    
    def state_callback(self, msg):
        """実際の位置の監視と速度計算"""
        current_time = rospy.Time.now()
        current_pos = [msg.x, msg.y, msg.z]
        
        # 位置の安全性チェック
        if not self.is_position_safe(msg.x, msg.y, msg.z):
            warning_msg = f"UNSAFE POSITION detected for {self.target_leg}: [{msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}]"
            rospy.logwarn(warning_msg)
            self.warning_pub.publish(String(data=warning_msg))
            self.publish_emergency_stop("Unsafe position detected")
            return
        
        # 速度計算（前回の位置がある場合）
        if self.last_position is not None and self.last_time is not None:
            dt = (current_time - self.last_time).to_sec()
            if dt > 0:
                velocity = self.calculate_velocity(self.last_position, current_pos, dt)
                if velocity > self.velocity_limit:
                    warning_msg = f"HIGH VELOCITY detected for {self.target_leg}: {velocity:.1f} mm/s"
                    rospy.logwarn(warning_msg)
                    self.warning_pub.publish(String(data=warning_msg))
        
        # 現在の状態を保存
        self.last_position = current_pos.copy()
        self.last_time = current_time
        
        # 定期的に状態をログ出力
        if rospy.Time.now().to_sec() % 5.0 < 0.1:  # 5秒毎
            rospy.loginfo(f"{self.target_leg} position: [{msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}]")
    
    def is_position_safe(self, x, y, z):
        """位置が安全範囲内かチェック"""
        return (self.safety_limits['x_min'] <= x <= self.safety_limits['x_max'] and
                self.safety_limits['y_min'] <= y <= self.safety_limits['y_max'] and
                self.safety_limits['z_min'] <= z <= self.safety_limits['z_max'])
    
    def calculate_velocity(self, pos1, pos2, dt):
        """2点間の速度を計算"""
        dx = pos2[0] - pos1[0]
        dy = pos2[1] - pos1[1]
        dz = pos2[2] - pos1[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        return distance / dt
    
    def publish_emergency_stop(self, reason):
        """緊急停止信号を発行"""
        emergency_msg = f"EMERGENCY STOP: {reason} for leg {self.target_leg}"
        rospy.logerr(emergency_msg)
        self.emergency_pub.publish(String(data=emergency_msg))

if __name__ == '__main__':
    try:
        monitor = LegSafetyMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Safety monitor shutting down")
#!/usr/bin/env python3
"""
滑らかな遷移状態監視ノード
方向変更や速度変更の状態をリアルタイムで表示
"""

import rospy
import math
from dual_leg_controller.msg import LegPosition
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SmoothTransitionMonitor:
    def __init__(self):
        rospy.init_node('smooth_transition_monitor', anonymous=True)
        
        # パラメータ取得
        self.update_rate = rospy.get_param('~update_rate', 10.0)
        self.monitor_legs = ["RF", "LF", "LM", "LB", "RB", "RM"]
        
        # 状態変数
        self.leg_positions = {}
        self.cmd_vel_data = None
        self.last_positions = {}
        self.velocities = {}
        
        # 統計情報
        self.max_velocity = 0.0
        self.direction_changes = 0
        self.last_direction = None
        
        # Subscribers
        for leg_id in self.monitor_legs:
            topic = f'/asterisk/leg/{leg_id}/command/foot_position'
            rospy.Subscriber(topic, LegPosition, 
                           lambda msg, leg=leg_id: self.position_callback(msg, leg))
        
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Timer for status display
        rospy.Timer(rospy.Duration(1.0/self.update_rate), self.display_status)
        
        rospy.loginfo("Smooth Transition Monitor started")
        rospy.loginfo(f"Monitoring legs: {', '.join(self.monitor_legs)}")
        
    def position_callback(self, msg, leg_id):
        """脚位置の更新"""
        current_pos = [msg.x, msg.y, msg.z]
        current_time = rospy.Time.now()
        
        # 速度計算
        if leg_id in self.last_positions:
            last_pos, last_time = self.last_positions[leg_id]
            dt = (current_time - last_time).to_sec()
            
            if dt > 0:
                dx = current_pos[0] - last_pos[0]
                dy = current_pos[1] - last_pos[1]
                dz = current_pos[2] - last_pos[2]
                velocity = math.sqrt(dx*dx + dy*dy + dz*dz) / dt
                
                self.velocities[leg_id] = velocity
                if velocity > self.max_velocity:
                    self.max_velocity = velocity
        
        self.leg_positions[leg_id] = current_pos
        self.last_positions[leg_id] = (current_pos, current_time)
    
    def cmd_vel_callback(self, msg):
        """cmd_velの監視"""
        self.cmd_vel_data = msg
        
        # 方向変更の検出
        current_direction = math.atan2(msg.linear.y, msg.linear.x) * 180.0 / math.pi
        if self.last_direction is not None:
            direction_diff = abs(current_direction - self.last_direction)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff
            
            if direction_diff > 10.0:  # 10度以上の変更
                self.direction_changes += 1
        
        self.last_direction = current_direction
    
    def display_status(self, event):
        """状態表示"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("SMOOTH TRANSITION MONITOR STATUS")
        rospy.loginfo("=" * 60)
        
        # cmd_vel情報
        if self.cmd_vel_data:
            speed = math.sqrt(self.cmd_vel_data.linear.x**2 + self.cmd_vel_data.linear.y**2) * 1000
            direction = math.atan2(self.cmd_vel_data.linear.y, self.cmd_vel_data.linear.x) * 180.0 / math.pi
            rospy.loginfo(f"CMD_VEL: Speed={speed:.1f}mm/s, Direction={direction:.1f}°, Angular={self.cmd_vel_data.angular.z:.2f}rad/s")
        
        # 脚位置の範囲
        if len(self.leg_positions) >= 6:
            all_x = [pos[0] for pos in self.leg_positions.values()]
            all_y = [pos[1] for pos in self.leg_positions.values()]
            all_z = [pos[2] for pos in self.leg_positions.values()]
            
            rospy.loginfo(f"POSITION RANGES:")
            rospy.loginfo(f"  X: {min(all_x):.1f} ~ {max(all_x):.1f} mm (spread: {max(all_x)-min(all_x):.1f}mm)")
            rospy.loginfo(f"  Y: {min(all_y):.1f} ~ {max(all_y):.1f} mm (spread: {max(all_y)-min(all_y):.1f}mm)")
            rospy.loginfo(f"  Z: {min(all_z):.1f} ~ {max(all_z):.1f} mm (spread: {max(all_z)-min(all_z):.1f}mm)")
        
        # 脚の個別速度
        if self.velocities:
            rospy.loginfo(f"LEG VELOCITIES:")
            for leg_id in sorted(self.velocities.keys()):
                velocity = self.velocities[leg_id]
                status = "🔶" if velocity > 50 else "🟢" if velocity > 10 else "⚪"
                rospy.loginfo(f"  {leg_id}: {velocity:.1f}mm/s {status}")
        
        # 統計情報
        rospy.loginfo(f"STATISTICS:")
        rospy.loginfo(f"  Max velocity recorded: {self.max_velocity:.1f}mm/s")
        rospy.loginfo(f"  Direction changes: {self.direction_changes}")
        
        # 滑らかさの評価
        if self.velocities:
            avg_velocity = sum(self.velocities.values()) / len(self.velocities)
            velocity_variance = sum((v - avg_velocity)**2 for v in self.velocities.values()) / len(self.velocities)
            smoothness_score = max(0, 100 - velocity_variance)
            
            rospy.loginfo(f"SMOOTHNESS ASSESSMENT:")
            rospy.loginfo(f"  Average velocity: {avg_velocity:.1f}mm/s")
            rospy.loginfo(f"  Velocity variance: {velocity_variance:.1f}")
            rospy.loginfo(f"  Smoothness score: {smoothness_score:.1f}/100")
            
            if smoothness_score > 80:
                rospy.loginfo("  Status: ✅ VERY SMOOTH")
            elif smoothness_score > 60:
                rospy.loginfo("  Status: 🟡 MODERATELY SMOOTH")
            else:
                rospy.loginfo("  Status: 🔴 JERKY MOTION DETECTED")
        
        # 推奨設定
        if self.max_velocity > 100:
            rospy.logwarn("⚠️  High velocities detected - consider reducing direction_change_rate")
        
        rospy.loginfo("=" * 60)
    
    def get_direction_name(self, angle_deg):
        """角度から方向名を取得"""
        angle_deg = angle_deg % 360
        directions = [
            (0, "Forward"), (45, "Forward-Right"), (90, "Right"), (135, "Backward-Right"),
            (180, "Backward"), (225, "Backward-Left"), (270, "Left"), (315, "Forward-Left")
        ]
        
        for angle, name in directions:
            if abs(angle_deg - angle) < 22.5:
                return name
        return "Forward"  # Default

if __name__ == '__main__':
    try:
        monitor = SmoothTransitionMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Smooth Transition Monitor shutting down")
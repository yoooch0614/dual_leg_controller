#!/usr/bin/env python3
"""
6脚ロボット足先動作監視システム
リアルタイムで全6脚の足先位置、軌道、速度、歩行パターンを監視・可視化
"""

import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Polygon
from dual_leg_controller.msg import LegPosition, LegCommand
from std_msgs.msg import String
import threading
import time
from collections import deque
import json

class HexapodFootMonitor:
    def __init__(self):
        rospy.init_node('hexapod_foot_monitor', anonymous=True)
        
        # 監視パラメータ
        self.leg_ids = ["RF", "LF", "LM", "LB", "RB", "RM"]
        self.leg_colors = {
            "RF": "red", "LF": "blue", "LM": "green", 
            "LB": "orange", "RB": "purple", "RM": "brown"
        }
        
        # 脚の取り付け角度（度）
        self.leg_angles = {
            "RF": 0, "LF": 300, "LM": 240, 
            "LB": 180, "RB": 120, "RM": 60
        }
        
        # トライポッドグループ
        self.tripod_groups = {
            "RF": 0, "LF": 1, "LM": 0, 
            "LB": 1, "RB": 0, "RM": 1
        }
        
        # データ保存
        self.foot_positions = {leg_id: deque(maxlen=100) for leg_id in self.leg_ids}
        self.foot_velocities = {leg_id: deque(maxlen=50) for leg_id in self.leg_ids}
        self.foot_timestamps = {leg_id: deque(maxlen=100) for leg_id in self.leg_ids}
        self.current_positions = {}
        self.last_positions = {}
        self.last_times = {}
        
        # 統計情報
        self.stats = {
            "max_positions": {leg_id: {"x": -999, "y": -999, "z": -999} for leg_id in self.leg_ids},
            "min_positions": {leg_id: {"x": 999, "y": 999, "z": 999} for leg_id in self.leg_ids},
            "max_velocities": {leg_id: 0.0 for leg_id in self.leg_ids},
            "step_counts": {leg_id: 0 for leg_id in self.leg_ids},
            "ground_contact_time": {leg_id: 0.0 for leg_id in self.leg_ids},
            "swing_time": {leg_id: 0.0 for leg_id in self.leg_ids}
        }
        
        # 異常検出
        self.anomaly_thresholds = {
            "max_velocity": 200.0,     # mm/s
            "max_position_x": 250.0,   # mm
            "min_position_x": 50.0,    # mm
            "max_position_z": -30.0,   # mm
            "min_position_z": -150.0,  # mm
            "max_step_height": 50.0    # mm
        }
        
        self.anomalies = []
        
        # 歩行状態検出
        self.gait_analysis = {
            "phase_detection": {leg_id: "unknown" for leg_id in self.leg_ids},
            "step_frequency": {leg_id: 0.0 for leg_id in self.leg_ids},
            "coordination_quality": 0.0,
            "stability_margin": 0.0
        }
        
        # Subscribers
        self.setup_subscribers()
        
        # 表示用タイマー
        rospy.Timer(rospy.Duration(1.0), self.print_summary)
        rospy.Timer(rospy.Duration(0.1), self.analyze_gait)
        
        # 可視化の初期化
        self.setup_visualization()
        
        rospy.loginfo("Hexapod Foot Monitor initialized")
        rospy.loginfo("Monitoring legs: " + ", ".join(self.leg_ids))
        rospy.loginfo("Starting real-time visualization...")
        
    def setup_subscribers(self):
        """全脚の位置情報購読設定"""
        for leg_id in self.leg_ids:
            # 実際の足先位置（状態）
            state_topic = f'/asterisk/leg/{leg_id}/state/foot_position'
            rospy.Subscriber(state_topic, LegPosition, 
                           lambda msg, leg=leg_id: self.position_callback(msg, leg, "state"))
            
            # 指令位置
            cmd_topic = f'/asterisk/leg/{leg_id}/command/foot_position'
            rospy.Subscriber(cmd_topic, LegPosition, 
                           lambda msg, leg=leg_id: self.position_callback(msg, leg, "command"))
            
            rospy.loginfo(f"Subscribed to {leg_id} topics")
    
    def position_callback(self, msg, leg_id, msg_type):
        """足先位置データの処理"""
        current_time = rospy.Time.now()
        position = np.array([msg.x, msg.y, msg.z])
        
        # データ保存
        self.foot_positions[leg_id].append(position)
        self.foot_timestamps[leg_id].append(current_time.to_sec())
        self.current_positions[leg_id] = position
        
        # 速度計算
        if leg_id in self.last_positions and leg_id in self.last_times:
            dt = (current_time - self.last_times[leg_id]).to_sec()
            if dt > 0:
                velocity = np.linalg.norm(position - self.last_positions[leg_id]) / dt
                self.foot_velocities[leg_id].append(velocity)
                
                # 統計更新
                if velocity > self.stats["max_velocities"][leg_id]:
                    self.stats["max_velocities"][leg_id] = velocity
        
        # 位置統計の更新
        for i, axis in enumerate(["x", "y", "z"]):
            if position[i] > self.stats["max_positions"][leg_id][axis]:
                self.stats["max_positions"][leg_id][axis] = position[i]
            if position[i] < self.stats["min_positions"][leg_id][axis]:
                self.stats["min_positions"][leg_id][axis] = position[i]
        
        # 異常検出
        self.detect_anomalies(leg_id, position, msg_type)
        
        # 前回データの更新
        self.last_positions[leg_id] = position
        self.last_times[leg_id] = current_time
    
    def detect_anomalies(self, leg_id, position, msg_type):
        """異常動作の検出"""
        anomalies_found = []
        
        # 位置範囲チェック
        if position[0] > self.anomaly_thresholds["max_position_x"]:
            anomalies_found.append(f"{leg_id}: X位置が異常に大きい ({position[0]:.1f}mm)")
        if position[0] < self.anomaly_thresholds["min_position_x"]:
            anomalies_found.append(f"{leg_id}: X位置が異常に小さい ({position[0]:.1f}mm)")
        if position[2] > self.anomaly_thresholds["max_position_z"]:
            anomalies_found.append(f"{leg_id}: Z位置が異常に高い ({position[2]:.1f}mm)")
        if position[2] < self.anomaly_thresholds["min_position_z"]:
            anomalies_found.append(f"{leg_id}: Z位置が異常に低い ({position[2]:.1f}mm)")
        
        # 速度チェック
        if leg_id in self.foot_velocities and len(self.foot_velocities[leg_id]) > 0:
            current_velocity = self.foot_velocities[leg_id][-1]
            if current_velocity > self.anomaly_thresholds["max_velocity"]:
                anomalies_found.append(f"{leg_id}: 速度が異常に高い ({current_velocity:.1f}mm/s)")
        
        # 新しい異常があれば記録
        for anomaly in anomalies_found:
            if anomaly not in [a["description"] for a in self.anomalies[-10:]]:
                self.anomalies.append({
                    "time": rospy.Time.now().to_sec(),
                    "leg_id": leg_id,
                    "description": anomaly,
                    "position": position.tolist(),
                    "type": msg_type
                })
                rospy.logwarn(f"🚨 ANOMALY DETECTED: {anomaly}")
    
    def analyze_gait(self, event):
        """歩行パターンの分析"""
        if len(self.current_positions) < 6:
            return
        
        # 各脚の位相検出（地面接触 vs スイング）
        for leg_id in self.leg_ids:
            if leg_id in self.current_positions:
                z_pos = self.current_positions[leg_id][2]
                # Z座標による地面接触判定（-95mm以下を地面接触とする）
                if z_pos <= -85.0:
                    self.gait_analysis["phase_detection"][leg_id] = "stance"
                else:
                    self.gait_analysis["phase_detection"][leg_id] = "swing"
        
        # トライポッド歩行の協調性チェック
        group0_stance = sum(1 for leg_id in ["RF", "LM", "RB"] 
                           if self.gait_analysis["phase_detection"].get(leg_id) == "stance")
        group1_stance = sum(1 for leg_id in ["LF", "LB", "RM"] 
                           if self.gait_analysis["phase_detection"].get(leg_id) == "stance")
        
        # 理想的なトライポッド歩行では、片方のグループが3脚全て接地、もう片方が0脚接地
        ideal_coordination = (group0_stance == 3 and group1_stance == 0) or \
                            (group0_stance == 0 and group1_stance == 3)
        
        if ideal_coordination:
            self.gait_analysis["coordination_quality"] = min(100.0, 
                                                           self.gait_analysis["coordination_quality"] + 1.0)
        else:
            self.gait_analysis["coordination_quality"] = max(0.0, 
                                                           self.gait_analysis["coordination_quality"] - 0.5)
    
    def setup_visualization(self):
        """リアルタイム可視化の設定"""
        self.fig = plt.figure(figsize=(15, 10))
        
        # サブプロット作成
        self.ax_3d = self.fig.add_subplot(221, projection='3d')
        self.ax_xy = self.fig.add_subplot(222)
        self.ax_velocity = self.fig.add_subplot(223)
        self.ax_phase = self.fig.add_subplot(224)
        
        # 3D軌道プロット
        self.ax_3d.set_xlabel('X (mm)')
        self.ax_3d.set_ylabel('Y (mm)')
        self.ax_3d.set_zlabel('Z (mm)')
        self.ax_3d.set_title('3D足先軌道')
        
        # XY平面プロット
        self.ax_xy.set_xlabel('X (mm)')
        self.ax_xy.set_ylabel('Y (mm)')
        self.ax_xy.set_title('XY平面軌道（上面図）')
        self.ax_xy.set_aspect('equal')
        
        # 速度プロット
        self.ax_velocity.set_xlabel('時間')
        self.ax_velocity.set_ylabel('速度 (mm/s)')
        self.ax_velocity.set_title('足先速度')
        
        # 歩行位相プロット
        self.ax_phase.set_title('歩行位相状態')
        self.ax_phase.set_ylim(-0.5, 5.5)
        
        # ボディの描画（六角形）
        body_radius = 95.0
        body_angles = np.linspace(0, 2*np.pi, 7)
        body_x = body_radius * np.cos(body_angles)
        body_y = body_radius * np.sin(body_angles)
        self.ax_xy.plot(body_x, body_y, 'k-', linewidth=2, label='Robot Body')
        
        # 脚の取り付け位置の表示
        for leg_id in self.leg_ids:
            angle_rad = math.radians(self.leg_angles[leg_id])
            attach_x = body_radius * math.cos(angle_rad)
            attach_y = body_radius * math.sin(angle_rad)
            self.ax_xy.plot(attach_x, attach_y, 'ko', markersize=8)
            self.ax_xy.text(attach_x + 10, attach_y + 10, leg_id, fontsize=8)
        
        self.ax_xy.grid(True)
        self.ax_xy.legend()
        
        # アニメーション開始
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, 
                                         interval=100, blit=False)
        
        # 別スレッドでmatplotlibを実行
        self.plot_thread = threading.Thread(target=self.show_plot)
        self.plot_thread.daemon = True
        self.plot_thread.start()
    
    def update_plot(self, frame):
        """プロットの更新"""
        # 軸をクリア
        self.ax_3d.clear()
        self.ax_xy.clear()
        self.ax_velocity.clear()
        self.ax_phase.clear()
        
        # 3D軌道の描画
        self.ax_3d.set_xlabel('X (mm)')
        self.ax_3d.set_ylabel('Y (mm)')
        self.ax_3d.set_zlabel('Z (mm)')
        self.ax_3d.set_title('3D足先軌道（最近100点）')
        
        for leg_id in self.leg_ids:
            if len(self.foot_positions[leg_id]) > 1:
                positions = np.array(list(self.foot_positions[leg_id]))
                self.ax_3d.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                               color=self.leg_colors[leg_id], label=leg_id, alpha=0.7)
                
                # 現在位置をマーク
                if len(positions) > 0:
                    current = positions[-1]
                    self.ax_3d.scatter(current[0], current[1], current[2], 
                                     color=self.leg_colors[leg_id], s=100, marker='o')
        
        self.ax_3d.legend()
        self.ax_3d.set_xlim([50, 250])
        self.ax_3d.set_ylim([-100, 100])
        self.ax_3d.set_zlim([-150, -50])
        
        # XY平面軌道の描画
        self.ax_xy.set_xlabel('X (mm)')
        self.ax_xy.set_ylabel('Y (mm)')
        self.ax_xy.set_title('XY平面軌道（上面図）')
        self.ax_xy.set_aspect('equal')
        
        # ボディの再描画
        body_radius = 95.0
        body_angles = np.linspace(0, 2*np.pi, 7)
        body_x = body_radius * np.cos(body_angles)
        body_y = body_radius * np.sin(body_angles)
        self.ax_xy.plot(body_x, body_y, 'k-', linewidth=2, alpha=0.5)
        
        for leg_id in self.leg_ids:
            if len(self.foot_positions[leg_id]) > 1:
                positions = np.array(list(self.foot_positions[leg_id]))
                self.ax_xy.plot(positions[:, 0], positions[:, 1], 
                               color=self.leg_colors[leg_id], label=leg_id, alpha=0.7)
                
                # 現在位置
                if len(positions) > 0:
                    current = positions[-1]
                    phase = self.gait_analysis["phase_detection"].get(leg_id, "unknown")
                    marker = 'o' if phase == "stance" else '^'
                    self.ax_xy.scatter(current[0], current[1], 
                                     color=self.leg_colors[leg_id], s=100, marker=marker)
        
        self.ax_xy.grid(True)
        self.ax_xy.legend()
        self.ax_xy.set_xlim([50, 250])
        self.ax_xy.set_ylim([-100, 100])
        
        # 速度プロットの描画
        self.ax_velocity.set_xlabel('データポイント')
        self.ax_velocity.set_ylabel('速度 (mm/s)')
        self.ax_velocity.set_title('足先速度（最近50点）')
        
        for leg_id in self.leg_ids:
            if len(self.foot_velocities[leg_id]) > 1:
                velocities = list(self.foot_velocities[leg_id])
                self.ax_velocity.plot(velocities, color=self.leg_colors[leg_id], 
                                    label=f"{leg_id} ({velocities[-1]:.1f}mm/s)", linewidth=2)
        
        self.ax_velocity.legend()
        self.ax_velocity.grid(True)
        self.ax_velocity.axhline(y=100, color='red', linestyle='--', alpha=0.5, label='High Speed Warning')
        
        # 歩行位相の描画
        self.ax_phase.set_title('歩行位相状態（●=接地, ▲=スイング）')
        self.ax_phase.set_ylim(-0.5, 5.5)
        self.ax_phase.set_xlim(-1, 1)
        
        for i, leg_id in enumerate(self.leg_ids):
            phase = self.gait_analysis["phase_detection"].get(leg_id, "unknown")
            color = self.leg_colors[leg_id]
            
            if phase == "stance":
                self.ax_phase.scatter(0, i, color=color, s=200, marker='o', label=f"{leg_id} 接地")
            elif phase == "swing":
                self.ax_phase.scatter(0, i, color=color, s=200, marker='^', label=f"{leg_id} スイング")
            else:
                self.ax_phase.scatter(0, i, color='gray', s=100, marker='?', label=f"{leg_id} 不明")
            
            self.ax_phase.text(0.2, i, leg_id, fontsize=10, va='center')
        
        self.ax_phase.set_yticks(range(6))
        self.ax_phase.set_yticklabels(self.leg_ids)
        self.ax_phase.set_xticks([])
        
        # 協調性スコアの表示
        coord_quality = self.gait_analysis["coordination_quality"]
        self.ax_phase.text(0.5, 5.2, f"協調性: {coord_quality:.1f}%", 
                          fontsize=12, weight='bold',
                          color='green' if coord_quality > 70 else 'orange' if coord_quality > 40 else 'red')
        
        plt.tight_layout()
    
    def show_plot(self):
        """matplotlibの表示"""
        plt.show()
    
    def print_summary(self, event):
        """定期的なサマリー表示"""
        rospy.loginfo("=" * 80)
        rospy.loginfo("📊 HEXAPOD FOOT MONITOR SUMMARY")
        rospy.loginfo("=" * 80)
        
        # 基本統計
        rospy.loginfo("📍 POSITION RANGES:")
        for leg_id in self.leg_ids:
            max_pos = self.stats["max_positions"][leg_id]
            min_pos = self.stats["min_positions"][leg_id]
            rospy.loginfo(f"  {leg_id}: X[{min_pos['x']:.1f}~{max_pos['x']:.1f}] "
                         f"Y[{min_pos['y']:.1f}~{max_pos['y']:.1f}] "
                         f"Z[{min_pos['z']:.1f}~{max_pos['z']:.1f}]")
        
        # 速度統計
        rospy.loginfo("🏃 VELOCITY STATS:")
        for leg_id in self.leg_ids:
            max_vel = self.stats["max_velocities"][leg_id]
            current_vel = self.foot_velocities[leg_id][-1] if self.foot_velocities[leg_id] else 0.0
            status = "🔴" if max_vel > 150 else "🟡" if max_vel > 100 else "🟢"
            rospy.loginfo(f"  {leg_id}: Current={current_vel:.1f}mm/s, Max={max_vel:.1f}mm/s {status}")
        
        # 歩行位相状態
        rospy.loginfo("👣 GAIT PHASE:")
        tripod0_status = []
        tripod1_status = []
        
        for leg_id in self.leg_ids:
            phase = self.gait_analysis["phase_detection"].get(leg_id, "unknown")
            symbol = "●" if phase == "stance" else "▲" if phase == "swing" else "?"
            
            if self.tripod_groups[leg_id] == 0:
                tripod0_status.append(f"{leg_id}{symbol}")
            else:
                tripod1_status.append(f"{leg_id}{symbol}")
        
        rospy.loginfo(f"  Tripod 0: {' '.join(tripod0_status)}")
        rospy.loginfo(f"  Tripod 1: {' '.join(tripod1_status)}")
        rospy.loginfo(f"  Coordination Quality: {self.gait_analysis['coordination_quality']:.1f}%")
        
        # 異常検出
        if self.anomalies:
            recent_anomalies = [a for a in self.anomalies if (rospy.Time.now().to_sec() - a["time"]) < 10.0]
            if recent_anomalies:
                rospy.loginfo("🚨 RECENT ANOMALIES (last 10 seconds):")
                for anomaly in recent_anomalies[-5:]:  # 最新5件
                    rospy.loginfo(f"  {anomaly['description']}")
            else:
                rospy.loginfo("✅ NO RECENT ANOMALIES")
        else:
            rospy.loginfo("✅ NO ANOMALIES DETECTED")
        
        # データ取得状況
        rospy.loginfo("📡 DATA COLLECTION STATUS:")
        for leg_id in self.leg_ids:
            data_points = len(self.foot_positions[leg_id])
            status = "🟢" if data_points > 50 else "🟡" if data_points > 10 else "🔴"
            rospy.loginfo(f"  {leg_id}: {data_points} data points {status}")
        
        rospy.loginfo("=" * 80)
    
    def save_data(self, filename=None):
        """データの保存"""
        if filename is None:
            filename = f"hexapod_foot_data_{int(time.time())}.json"
        
        data = {
            "timestamp": rospy.Time.now().to_sec(),
            "stats": self.stats,
            "anomalies": self.anomalies,
            "gait_analysis": self.gait_analysis,
            "foot_positions": {leg_id: [pos.tolist() for pos in list(self.foot_positions[leg_id])] 
                              for leg_id in self.leg_ids}
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        rospy.loginfo(f"Data saved to {filename}")

if __name__ == '__main__':
    try:
        monitor = HexapodFootMonitor()
        rospy.loginfo("Hexapod Foot Monitor is running...")
        rospy.loginfo("Press Ctrl+C to stop")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Hexapod Foot Monitor shutting down")
    except Exception as e:
        rospy.logerr(f"Error in Hexapod Foot Monitor: {e}")
        import traceback
        traceback.print_exc()
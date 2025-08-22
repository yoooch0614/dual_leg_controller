#!/usr/bin/env python3
"""
6脚ロボット歩行パターン分析システム（完全版）
トライポッド歩行の品質、協調性、安定性を詳細分析
"""

import rospy
import numpy as np
import math
from collections import deque, defaultdict
from dual_leg_controller.msg import LegPosition
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point, PolygonStamped
import json

class GaitPatternAnalyzer:
    def __init__(self):
        rospy.init_node('gait_pattern_analyzer', anonymous=True)
        
        # 設定パラメータ
        self.leg_ids = ["RF", "LF", "LM", "LB", "RB", "RM"]
        self.tripod_groups = {
            "RF": 0, "LM": 0, "RB": 0,  # グループ0
            "LF": 1, "LB": 1, "RM": 1   # グループ1
        }
        
        # 分析パラメータ
        self.analysis_window = rospy.get_param('~analysis_window_size', 5.0)  # 秒
        self.stance_threshold = rospy.get_param('~stance_z_threshold', -85.0)  # mm
        self.coordination_threshold = rospy.get_param('~tripod_coordination_threshold', 0.7)
        
        # データ保存
        self.foot_positions = {leg_id: deque(maxlen=500) for leg_id in self.leg_ids}
        self.timestamps = {leg_id: deque(maxlen=500) for leg_id in self.leg_ids}
        self.phase_history = {leg_id: deque(maxlen=200) for leg_id in self.leg_ids}
        
        # 分析結果
        self.gait_metrics = {
            'coordination_score': 0.0,
            'stability_margin': 0.0,
            'step_regularity': 0.0,
            'phase_consistency': 0.0,
            'duty_factor': {leg_id: 0.0 for leg_id in self.leg_ids},
            'step_frequency': 0.0,
            'stride_length': 0.0,
            'gait_efficiency': 0.0
        }
        
        # ステップ検出
        self.step_events = {leg_id: [] for leg_id in self.leg_ids}
        self.last_phases = {leg_id: 'unknown' for leg_id in self.leg_ids}
        
        # Publishers
        self.coordination_pub = rospy.Publisher('/gait_analysis/coordination_score', Float64, queue_size=1)
        self.stability_pub = rospy.Publisher('/gait_analysis/stability_margin', Float64, queue_size=1)
        self.efficiency_pub = rospy.Publisher('/gait_analysis/efficiency', Float64, queue_size=1)
        self.report_pub = rospy.Publisher('/gait_analysis/report', String, queue_size=1)
        self.support_polygon_pub = rospy.Publisher('/gait_analysis/support_polygon', PolygonStamped, queue_size=1)
        
        # Subscribers
        self.setup_subscribers()
        
        # タイマー
        rospy.Timer(rospy.Duration(0.1), self.analyze_gait)  # 10Hz分析
        rospy.Timer(rospy.Duration(2.0), self.publish_report)  # 2秒毎レポート
        
        rospy.loginfo("Gait Pattern Analyzer initialized")
        
    def setup_subscribers(self):
        """足先位置情報の購読設定"""
        for leg_id in self.leg_ids:
            topic = f'/asterisk/leg/{leg_id}/command/foot_position'
            rospy.Subscriber(topic, LegPosition, 
                           lambda msg, leg=leg_id: self.position_callback(msg, leg))
            
    def position_callback(self, msg, leg_id):
        """足先位置データの処理"""
        current_time = rospy.Time.now().to_sec()
        position = np.array([msg.x, msg.y, msg.z])
        
        self.foot_positions[leg_id].append(position)
        self.timestamps[leg_id].append(current_time)
        
        # 位相検出
        phase = self.detect_phase(position[2])
        self.phase_history[leg_id].append({
            'time': current_time,
            'phase': phase,
            'position': position
        })
        
        # ステップイベント検出
        self.detect_step_events(leg_id, phase, current_time, position)
        
        self.last_phases[leg_id] = phase
        
    def detect_phase(self, z_position):
        """Z座標から歩行位相を検出"""
        if z_position <= self.stance_threshold:
            return 'stance'
        else:
            return 'swing'
            
    def detect_step_events(self, leg_id, current_phase, time, position):
        """ステップイベント（接地・離地）の検出"""
        if leg_id not in self.last_phases:
            return
            
        last_phase = self.last_phases[leg_id]
        
        # 位相変化の検出
        if last_phase != current_phase:
            event_type = None
            if last_phase == 'swing' and current_phase == 'stance':
                event_type = 'touchdown'
            elif last_phase == 'stance' and current_phase == 'swing':
                event_type = 'liftoff'
                
            if event_type:
                self.step_events[leg_id].append({
                    'time': time,
                    'type': event_type,
                    'position': position.copy()
                })
                
                # 古いイベントの削除（10秒以上前）
                self.step_events[leg_id] = [
                    event for event in self.step_events[leg_id] 
                    if time - event['time'] < 10.0
                ]
                
    def analyze_gait(self, event):
        """歩行パターンの総合分析"""
        current_time = rospy.Time.now().to_sec()
        
        # 十分なデータがあるかチェック
        if not all(len(self.phase_history[leg_id]) > 10 for leg_id in self.leg_ids):
            return
            
        # 1. 協調性分析
        self.gait_metrics['coordination_score'] = self.analyze_coordination()
        
        # 2. 安定性解析
        self.gait_metrics['stability_margin'] = self.analyze_stability()
        
        # 3. ステップ規則性分析
        self.gait_metrics['step_regularity'] = self.analyze_step_regularity()
        
        # 4. デューティファクター計算
        self.calculate_duty_factors()
        
        # 5. ステップ頻度計算
        self.gait_metrics['step_frequency'] = self.calculate_step_frequency()
        
        # 6. 歩行効率計算
        self.gait_metrics['gait_efficiency'] = self.calculate_gait_efficiency()
        
        # 7. 位相一貫性分析
        self.gait_metrics['phase_consistency'] = self.analyze_phase_consistency()
        
        # パブリッシュ
        self.coordination_pub.publish(Float64(data=self.gait_metrics['coordination_score']))
        self.stability_pub.publish(Float64(data=self.gait_metrics['stability_margin']))
        self.efficiency_pub.publish(Float64(data=self.gait_metrics['gait_efficiency']))
        
        # サポートポリゴンの計算と公開
        self.publish_support_polygon()
        
    def analyze_coordination(self):
        """トライポッド歩行の協調性分析"""
        coordination_scores = []
        
        # 過去の位相データを分析
        min_length = min(len(self.phase_history[leg_id]) for leg_id in self.leg_ids)
        
        for i in range(min(min_length, 50)):
            try:
                # 各脚の同時刻の位相を取得
                phases_at_time = {}
                for leg_id in self.leg_ids:
                    if len(self.phase_history[leg_id]) > i:
                        phases_at_time[leg_id] = self.phase_history[leg_id][-1-i]['phase']
                
                if len(phases_at_time) == 6:
                    # トライポッドグループごとの接地数をカウント
                    group0_stance = sum(1 for leg_id in ["RF", "LM", "RB"] 
                                      if phases_at_time.get(leg_id) == 'stance')
                    group1_stance = sum(1 for leg_id in ["LF", "LB", "RM"] 
                                      if phases_at_time.get(leg_id) == 'stance')
                    
                    # 理想的なトライポッド歩行のスコア計算
                    if (group0_stance == 3 and group1_stance == 0) or \
                       (group0_stance == 0 and group1_stance == 3):
                        coordination_scores.append(1.0)  # 完璧
                    elif (group0_stance == 2 and group1_stance == 1) or \
                         (group0_stance == 1 and group1_stance == 2):
                        coordination_scores.append(0.7)  # 良好
                    elif group0_stance == group1_stance:
                        coordination_scores.append(0.3)  # 普通
                    else:
                        coordination_scores.append(0.0)  # 悪い
                        
            except (IndexError, KeyError):
                continue
                
        if coordination_scores:
            return np.mean(coordination_scores) * 100.0
        return 0.0
        
    def analyze_stability(self):
        """静的安定余裕の計算"""
        # 現在接地している脚の位置を取得
        stance_legs = []
        for leg_id in self.leg_ids:
            if len(self.phase_history[leg_id]) > 0:
                current_phase = self.phase_history[leg_id][-1]['phase']
                if current_phase == 'stance':
                    position = self.phase_history[leg_id][-1]['position']
                    stance_legs.append(position[:2])  # X, Y座標のみ
        
        if len(stance_legs) < 3:
            return 0.0  # 静的に不安定
            
        # 重心位置（仮定：原点）
        center_of_mass = np.array([0.0, 0.0])
        
        # サポートポリゴンの計算
        if len(stance_legs) >= 3:
            stance_array = np.array(stance_legs)
            
            # 凸包の計算
            try:
                # scipy使用可能な場合
                from scipy.spatial import ConvexHull
                hull = ConvexHull(stance_array)
                polygon_vertices = stance_array[hull.vertices]
            except ImportError:
                # scipyが無い場合の簡易計算
                polygon_vertices = stance_array
            
            # 重心からポリゴン境界までの最短距離
            min_distance = float('inf')
            for i in range(len(polygon_vertices)):
                p1 = polygon_vertices[i]
                p2 = polygon_vertices[(i + 1) % len(polygon_vertices)]
                
                # 点から線分への距離
                distance = self.point_to_line_distance(center_of_mass, p1, p2)
                min_distance = min(min_distance, distance)
            
            return min_distance
        
        return 0.0
        
    def point_to_line_distance(self, point, line_start, line_end):
        """点から線分への最短距離"""
        line_vec = line_end - line_start
        point_vec = point - line_start
        
        line_len = np.linalg.norm(line_vec)
        if line_len == 0:
            return np.linalg.norm(point_vec)
            
        line_unitvec = line_vec / line_len
        proj_length = np.dot(point_vec, line_unitvec)
        
        if proj_length < 0:
            return np.linalg.norm(point_vec)
        elif proj_length > line_len:
            return np.linalg.norm(point - line_end)
        else:
            proj_point = line_start + proj_length * line_unitvec
            return np.linalg.norm(point - proj_point)
            
    def analyze_step_regularity(self):
        """ステップの規則性分析"""
        step_intervals = []
        
        for leg_id in self.leg_ids:
            # 接地イベントの間隔を計算
            touchdowns = [event for event in self.step_events[leg_id] 
                         if event['type'] == 'touchdown']
            
            if len(touchdowns) >= 3:
                intervals = []
                for i in range(1, len(touchdowns)):
                    interval = touchdowns[i]['time'] - touchdowns[i-1]['time']
                    intervals.append(interval)
                
                if intervals:
                    # 変動係数（CV）を計算（小さいほど規則的）
                    mean_interval = np.mean(intervals)
                    std_interval = np.std(intervals)
                    
                    if mean_interval > 0:
                        cv = std_interval / mean_interval
                        regularity = max(0, 1.0 - cv)  # CVが小さいほど高いスコア
                        step_intervals.append(regularity)
        
        if step_intervals:
            return np.mean(step_intervals) * 100.0
        return 0.0
        
    def calculate_duty_factors(self):
        """各脚のデューティファクター計算"""
        for leg_id in self.leg_ids:
            if len(self.step_events[leg_id]) >= 4:
                # 最近の完全なステップサイクルを分析
                events = self.step_events[leg_id][-10:]  # 最新10イベント
                
                stance_times = []
                cycle_times = []
                
                touchdown_times = [e['time'] for e in events if e['type'] == 'touchdown']
                liftoff_times = [e['time'] for e in events if e['type'] == 'liftoff']
                
                # ペアリング
                for i in range(min(len(touchdown_times)-1, len(liftoff_times))):
                    if i < len(liftoff_times) and i+1 < len(touchdown_times):
                        stance_time = liftoff_times[i] - touchdown_times[i]
                        cycle_time = touchdown_times[i+1] - touchdown_times[i]
                        
                        if stance_time > 0 and cycle_time > 0:
                            stance_times.append(stance_time)
                            cycle_times.append(cycle_time)
                
                if stance_times and cycle_times:
                    mean_stance = np.mean(stance_times)
                    mean_cycle = np.mean(cycle_times)
                    if mean_cycle > 0:
                        self.gait_metrics['duty_factor'][leg_id] = mean_stance / mean_cycle
                        
    def calculate_step_frequency(self):
        """ステップ頻度の計算"""
        all_frequencies = []
        
        for leg_id in self.leg_ids:
            touchdowns = [event for event in self.step_events[leg_id] 
                         if event['type'] == 'touchdown']
            
            if len(touchdowns) >= 3:
                recent_touchdowns = [td for td in touchdowns 
                                   if rospy.Time.now().to_sec() - td['time'] < self.analysis_window]
                
                if len(recent_touchdowns) >= 2:
                    time_span = recent_touchdowns[-1]['time'] - recent_touchdowns[0]['time']
                    if time_span > 0:
                        frequency = (len(recent_touchdowns) - 1) / time_span
                        all_frequencies.append(frequency)
        
        if all_frequencies:
            return np.mean(all_frequencies)
        return 0.0
        
    def calculate_gait_efficiency(self):
        """歩行効率の計算"""
        # 前進距離と足先移動距離の比で効率を評価
        total_foot_movement = 0
        total_forward_progress = 0
        
        for leg_id in self.leg_ids:
            if len(self.foot_positions[leg_id]) >= 10:
                positions = list(self.foot_positions[leg_id])[-10:]
                
                # 足先の総移動距離
                foot_distance = 0
                for i in range(1, len(positions)):
                    distance = np.linalg.norm(positions[i] - positions[i-1])
                    foot_distance += distance
                
                # 前進距離（X方向の正味移動）
                if len(positions) >= 2:
                    forward_distance = abs(positions[-1][0] - positions[0][0])
                    
                    total_foot_movement += foot_distance
                    total_forward_progress += forward_distance
        
        if total_foot_movement > 0:
            efficiency = total_forward_progress / total_foot_movement
            return min(100.0, efficiency * 100.0)
        return 0.0
        
    def analyze_phase_consistency(self):
        """位相の一貫性分析"""
        # トライポッドグループ内の位相同期性をチェック
        group_consistencies = []
        
        for group_id in [0, 1]:
            group_legs = [leg_id for leg_id, group in self.tripod_groups.items() 
                         if group == group_id]
            
            # 各グループ内の位相の一致度を計算
            phase_agreements = []
            
            # 最近のデータで位相の一致を分析
            min_length = min(len(self.phase_history[leg_id]) for leg_id in group_legs)
            
            for i in range(min(50, min_length)):
                phases_at_time = {}
                for leg_id in group_legs:
                    if len(self.phase_history[leg_id]) > i:
                        phases_at_time[leg_id] = self.phase_history[leg_id][-1-i]['phase']
                
                if len(phases_at_time) == len(group_legs):
                    # 同じ位相の脚の数をカウント
                    stance_count = sum(1 for phase in phases_at_time.values() if phase == 'stance')
                    swing_count = sum(1 for phase in phases_at_time.values() if phase == 'swing')
                    
                    # 全て同じ位相なら1.0、完全にバラバラなら0.0
                    max_agreement = max(stance_count, swing_count)
                    agreement = max_agreement / len(group_legs)
                    phase_agreements.append(agreement)
            
            if phase_agreements:
                group_consistencies.append(np.mean(phase_agreements))
        
        if group_consistencies:
            return np.mean(group_consistencies) * 100.0
        return 0.0

    def publish_support_polygon(self):
        """サポートポリゴンの公開"""
        # 現在接地している脚の位置を取得
        stance_positions = []
        for leg_id in self.leg_ids:
            if len(self.phase_history[leg_id]) > 0:
                current_phase = self.phase_history[leg_id][-1]['phase']
                if current_phase == 'stance':
                    position = self.phase_history[leg_id][-1]['position']
                    stance_positions.append(position[:2])  # X, Y座標のみ
        
        if len(stance_positions) >= 3:
            # PolygonStampedメッセージの作成
            polygon_msg = PolygonStamped()
            polygon_msg.header.stamp = rospy.Time.now()
            polygon_msg.header.frame_id = "base_link"
            
            try:
                from scipy.spatial import ConvexHull
                stance_array = np.array(stance_positions)
                hull = ConvexHull(stance_array)
                
                for vertex_idx in hull.vertices:
                    point = Point()
                    point.x = stance_array[vertex_idx][0] / 1000.0  # mm to m
                    point.y = stance_array[vertex_idx][1] / 1000.0  # mm to m
                    point.z = 0.0
                    polygon_msg.polygon.points.append(point)
                
                self.support_polygon_pub.publish(polygon_msg)
                
            except ImportError:
                rospy.logwarn_once("scipy not available for convex hull calculation")
            except Exception as e:
                rospy.logwarn(f"Failed to compute support polygon: {e}")

    def publish_report(self, event):
        """分析レポートの公開"""
        report = {
            'timestamp': rospy.Time.now().to_sec(),
            'metrics': self.gait_metrics,
            'assessment': self.generate_assessment(),
            'recommendations': self.generate_recommendations()
        }
        
        # JSON形式でレポートを公開
        report_json = json.dumps(report, indent=2)
        self.report_pub.publish(String(data=report_json))
        
        # コンソールに要約を表示
        self.print_summary()

    def generate_assessment(self):
        """歩行品質の総合評価"""
        metrics = self.gait_metrics
        
        # 各メトリクスのスコア化
        scores = {
            'coordination': min(100, max(0, metrics['coordination_score'])),
            'stability': min(100, max(0, metrics['stability_margin'] * 10)),  # mm → %変換
            'regularity': min(100, max(0, metrics['step_regularity'])),
            'efficiency': min(100, max(0, metrics['gait_efficiency'])),
            'consistency': min(100, max(0, metrics['phase_consistency']))
        }
        
        # 重み付き総合スコア
        weights = {
            'coordination': 0.3,
            'stability': 0.25,
            'regularity': 0.2,
            'efficiency': 0.15,
            'consistency': 0.1
        }
        
        overall_score = sum(scores[key] * weights[key] for key in scores.keys())
        
        # 評価レベルの決定
        if overall_score >= 80:
            level = "EXCELLENT"
            color = "🟢"
        elif overall_score >= 65:
            level = "GOOD"
            color = "🟡"
        elif overall_score >= 50:
            level = "FAIR"
            color = "🟠"
        else:
            level = "POOR"
            color = "🔴"
        
        return {
            'overall_score': overall_score,
            'level': level,
            'color': color,
            'individual_scores': scores
        }

    def generate_recommendations(self):
        """改善提案の生成"""
        recommendations = []
        metrics = self.gait_metrics
        
        # 協調性の問題
        if metrics['coordination_score'] < 70:
            recommendations.append({
                'category': 'coordination',
                'issue': '歩行協調性が低い',
                'suggestion': 'トライポッド歩行パターンの調整、位相オフセットの確認'
            })
        
        # 安定性の問題
        if metrics['stability_margin'] < 5.0:  # 5mm未満
            recommendations.append({
                'category': 'stability',
                'issue': '安定余裕が小さい',
                'suggestion': 'ステップ幅の調整、デューティファクターの増加'
            })
        
        # 規則性の問題
        if metrics['step_regularity'] < 60:
            recommendations.append({
                'category': 'regularity',
                'issue': 'ステップが不規則',
                'suggestion': 'サイクル時間の調整、モーター制御の精度向上'
            })
        
        # 効率の問題
        if metrics['gait_efficiency'] < 50:
            recommendations.append({
                'category': 'efficiency',
                'issue': '歩行効率が低い',
                'suggestion': 'ステップ長さの最適化、無駄な動きの削減'
            })
        
        # デューティファクターの問題
        duty_factors = list(metrics['duty_factor'].values())
        if duty_factors and (max(duty_factors) - min(duty_factors)) > 0.2:
            recommendations.append({
                'category': 'duty_factor',
                'issue': '脚間のデューティファクター差が大きい',
                'suggestion': '各脚の接地時間の均一化、キャリブレーション'
            })
        
        return recommendations

    def print_summary(self):
        """分析結果の要約表示"""
        rospy.loginfo("=" * 70)
        rospy.loginfo("🦾 GAIT PATTERN ANALYSIS SUMMARY")
        rospy.loginfo("=" * 70)
        
        metrics = self.gait_metrics
        assessment = self.generate_assessment()
        
        # 総合評価
        rospy.loginfo(f"📊 OVERALL ASSESSMENT: {assessment['color']} {assessment['level']} "
                     f"({assessment['overall_score']:.1f}/100)")
        
        # 個別メトリクス
        rospy.loginfo("📈 DETAILED METRICS:")
        rospy.loginfo(f"  Coordination:     {metrics['coordination_score']:.1f}% "
                     f"{'🟢' if metrics['coordination_score'] > 70 else '🟡' if metrics['coordination_score'] > 50 else '🔴'}")
        rospy.loginfo(f"  Stability Margin: {metrics['stability_margin']:.1f}mm "
                     f"{'🟢' if metrics['stability_margin'] > 10 else '🟡' if metrics['stability_margin'] > 5 else '🔴'}")
        rospy.loginfo(f"  Step Regularity:  {metrics['step_regularity']:.1f}% "
                     f"{'🟢' if metrics['step_regularity'] > 70 else '🟡' if metrics['step_regularity'] > 50 else '🔴'}")
        rospy.loginfo(f"  Gait Efficiency:  {metrics['gait_efficiency']:.1f}% "
                     f"{'🟢' if metrics['gait_efficiency'] > 60 else '🟡' if metrics['gait_efficiency'] > 40 else '🔴'}")
        rospy.loginfo(f"  Phase Consistency: {metrics['phase_consistency']:.1f}% "
                     f"{'🟢' if metrics['phase_consistency'] > 70 else '🟡' if metrics['phase_consistency'] > 50 else '🔴'}")
        rospy.loginfo(f"  Step Frequency:   {metrics['step_frequency']:.2f} Hz")
        
        # デューティファクター
        rospy.loginfo("🦵 DUTY FACTORS:")
        for leg_id in self.leg_ids:
            duty = metrics['duty_factor'][leg_id]
            status = "🟢" if 0.4 <= duty <= 0.7 else "🟡" if 0.3 <= duty <= 0.8 else "🔴"
            rospy.loginfo(f"  {leg_id}: {duty:.2f} {status}")
        
        # 現在の歩行位相状態
        rospy.loginfo("👟 CURRENT PHASE STATUS:")
        tripod0_legs = []
        tripod1_legs = []
        
        for leg_id in self.leg_ids:
            if len(self.phase_history[leg_id]) > 0:
                current_phase = self.phase_history[leg_id][-1]['phase']
                symbol = "●" if current_phase == "stance" else "▲"
                
                if self.tripod_groups[leg_id] == 0:
                    tripod0_legs.append(f"{leg_id}{symbol}")
                else:
                    tripod1_legs.append(f"{leg_id}{symbol}")
        
        rospy.loginfo(f"  Tripod 0: {' '.join(tripod0_legs)}")
        rospy.loginfo(f"  Tripod 1: {' '.join(tripod1_legs)}")
        
        # 推奨事項
        recommendations = self.generate_recommendations()
        if recommendations:
            rospy.loginfo("💡 RECOMMENDATIONS:")
            for rec in recommendations[:3]:  # 上位3件
                rospy.loginfo(f"  • {rec['issue']}: {rec['suggestion']}")
        else:
            rospy.loginfo("✅ NO ISSUES DETECTED - Gait pattern is optimal!")
        
        rospy.loginfo("=" * 70)

if __name__ == '__main__':
    try:
        analyzer = GaitPatternAnalyzer()
        rospy.loginfo("Gait Pattern Analyzer is running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Gait Pattern Analyzer shutting down")
    except Exception as e:
        rospy.logerr(f"Error in Gait Pattern Analyzer: {e}")
        import traceback
        traceback.print_exc()
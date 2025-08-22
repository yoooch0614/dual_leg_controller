#!/usr/bin/env python3
"""
Webベース6脚ロボット監視インターフェース
ブラウザから歩行状態をリアルタイム監視
"""

import rospy
import json
import threading
import time
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
from dual_leg_controller.msg import LegPosition
from std_msgs.msg import String, Float64
import numpy as np
from collections import deque

class WebMonitorServer:
    def __init__(self):
        rospy.init_node('web_monitor_server', anonymous=True)
        
        # Flask設定
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'hexapod_monitor_secret'
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # 監視データ
        self.leg_ids = ["RF", "LF", "LM", "LB", "RB", "RM"]
        self.foot_positions = {leg_id: deque(maxlen=100) for leg_id in self.leg_ids}
        self.foot_velocities = {leg_id: deque(maxlen=50) for leg_id in self.leg_ids}
        self.gait_metrics = {}
        self.system_status = {
            'is_walking': False,
            'coordination_score': 0.0,
            'stability_margin': 0.0,
            'anomaly_count': 0,
            'uptime': 0
        }
        
        # タイムスタンプ
        self.last_positions = {}
        self.last_times = {}
        self.start_time = time.time()
        
        # ルート設定
        self.setup_routes()
        
        # ROS Subscribers
        self.setup_subscribers()
        
        # データ送信スレッド
        self.data_thread = threading.Thread(target=self.data_sender_loop)
        self.data_thread.daemon = True
        self.data_thread.start()
        
        rospy.loginfo("Web Monitor Server initialized")
        
    def setup_routes(self):
        """Flaskルートの設定"""
        
        @self.app.route('/')
        def index():
            return render_template('hexapod_monitor.html')
        
        @self.app.route('/api/status')
        def get_status():
            return jsonify(self.system_status)
        
        @self.app.route('/api/positions')
        def get_positions():
            positions = {}
            for leg_id in self.leg_ids:
                if len(self.foot_positions[leg_id]) > 0:
                    positions[leg_id] = self.foot_positions[leg_id][-1].tolist()
                else:
                    positions[leg_id] = [0, 0, 0]
            return jsonify(positions)
        
        @self.app.route('/api/metrics')
        def get_metrics():
            return jsonify(self.gait_metrics)
        
        @self.socketio.on('connect')
        def handle_connect():
            rospy.loginfo("Client connected to web monitor")
            emit('status', {'message': 'Connected to Hexapod Monitor'})
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            rospy.loginfo("Client disconnected from web monitor")
    
    def setup_subscribers(self):
        """ROS Subscribersの設定"""
        # 足先位置
        for leg_id in self.leg_ids:
            topic = f'/asterisk/leg/{leg_id}/command/foot_position'
            rospy.Subscriber(topic, LegPosition, 
                           lambda msg, leg=leg_id: self.position_callback(msg, leg))
        
        # 歩行分析結果
        rospy.Subscriber('/gait_analysis/coordination_score', Float64, self.coordination_callback)
        rospy.Subscriber('/gait_analysis/stability_margin', Float64, self.stability_callback)
        rospy.Subscriber('/gait_analysis/report', String, self.gait_report_callback)
        
    def position_callback(self, msg, leg_id):
        """足先位置データの処理"""
        current_time = time.time()
        position = np.array([msg.x, msg.y, msg.z])
        
        self.foot_positions[leg_id].append(position)
        
        # 速度計算
        if leg_id in self.last_positions and leg_id in self.last_times:
            dt = current_time - self.last_times[leg_id]
            if dt > 0:
                velocity = np.linalg.norm(position - self.last_positions[leg_id]) / dt
                self.foot_velocities[leg_id].append(velocity)
        
        self.last_positions[leg_id] = position
        self.last_times[leg_id] = current_time
        
    def coordination_callback(self, msg):
        """協調性スコアの更新"""
        self.system_status['coordination_score'] = msg.data
        
    def stability_callback(self, msg):
        """安定性の更新"""
        self.system_status['stability_margin'] = msg.data
        
    def gait_report_callback(self, msg):
        """歩行分析レポートの更新"""
        try:
            self.gait_metrics = json.loads(msg.data)
        except json.JSONDecodeError:
            rospy.logwarn("Failed to parse gait analysis report")
    
    def data_sender_loop(self):
        """データ送信ループ"""
        while not rospy.is_shutdown():
            try:
                # システム状態の更新
                self.system_status['uptime'] = time.time() - self.start_time
                self.system_status['is_walking'] = any(
                    len(positions) > 0 and len(positions) > 10 
                    for positions in self.foot_positions.values()
                )
                
                # 現在位置データ
                current_positions = {}
                current_velocities = {}
                
                for leg_id in self.leg_ids:
                    if len(self.foot_positions[leg_id]) > 0:
                        current_positions[leg_id] = self.foot_positions[leg_id][-1].tolist()
                    else:
                        current_positions[leg_id] = [0, 0, 0]
                    
                    if len(self.foot_velocities[leg_id]) > 0:
                        current_velocities[leg_id] = float(self.foot_velocities[leg_id][-1])
                    else:
                        current_velocities[leg_id] = 0.0
                
                # 軌道データ（最新20点）
                trajectory_data = {}
                for leg_id in self.leg_ids:
                    positions = list(self.foot_positions[leg_id])[-20:]
                    trajectory_data[leg_id] = [pos.tolist() for pos in positions]
                
                # データをWebクライアントに送信
                data_package = {
                    'timestamp': time.time(),
                    'system_status': self.system_status,
                    'positions': current_positions,
                    'velocities': current_velocities,
                    'trajectories': trajectory_data,
                    'metrics': self.gait_metrics
                }
                
                self.socketio.emit('data_update', data_package)
                
            except Exception as e:
                rospy.logwarn(f"Error in data sender loop: {e}")
            
            time.sleep(0.1)  # 10Hz
    
    def run(self):
        """サーバー実行"""
        host = rospy.get_param('~host', '0.0.0.0')
        port = rospy.get_param('~port', 8080)
        debug = rospy.get_param('~debug', False)
        
        rospy.loginfo(f"Starting web monitor server on {host}:{port}")
        self.socketio.run(self.app, host=host, port=port, debug=debug)

def create_html_template():
    """HTMLテンプレートの作成"""
    html_content = """
<!DOCTYPE html>
<html lang="ja">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>6脚ロボット監視システム</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f0f2f5;
        }
        .container {
            max-width: 1600px;
            margin: 0 auto;
        }
        .header {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 20px;
            border-radius: 10px;
            margin-bottom: 20px;
            text-align: center;
        }
        .status-panel {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin-bottom: 20px;
        }
        .status-card {
            background: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
            text-align: center;
        }
        .status-value {
            font-size: 2em;
            font-weight: bold;
            margin: 10px 0;
        }
        .charts-container {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-bottom: 20px;
        }
        .chart-panel {
            background: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        }
        .leg-status {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 15px;
            margin-bottom: 20px;
        }
        .leg-card {
            background: white;
            padding: 15px;
            border-radius: 10px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        }
        .leg-position {
            font-family: monospace;
            font-size: 0.9em;
            margin: 5px 0;
        }
        .status-good { color: #28a745; }
        .status-warning { color: #ffc107; }
        .status-danger { color: #dc3545; }
        .connection-status {
            position: fixed;
            top: 10px;
            right: 10px;
            padding: 10px;
            border-radius: 5px;
            color: white;
            font-weight: bold;
        }
        .connected { background-color: #28a745; }
        .disconnected { background-color: #dc3545; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🦾 6脚ロボット リアルタイム監視システム</h1>
            <p>足先動作・歩行パターン・システム状態の総合監視</p>
        </div>
        
        <div id="connectionStatus" class="connection-status disconnected">
            🔴 接続中...
        </div>
        
        <div class="status-panel">
            <div class="status-card">
                <h3>🤖 歩行状態</h3>
                <div id="walkingStatus" class="status-value">待機中</div>
            </div>
            <div class="status-card">
                <h3>🎯 協調性</h3>
                <div id="coordinationScore" class="status-value">0%</div>
            </div>
            <div class="status-card">
                <h3>⚖️ 安定性</h3>
                <div id="stabilityMargin" class="status-value">0mm</div>
            </div>
            <div class="status-card">
                <h3>⏱️ 稼働時間</h3>
                <div id="uptime" class="status-value">0s</div>
            </div>
        </div>
        
        <div class="charts-container">
            <div class="chart-panel">
                <h3>3D足先軌道</h3>
                <div id="trajectory3d" style="height: 400px;"></div>
            </div>
            <div class="chart-panel">
                <h3>XY平面軌道</h3>
                <div id="trajectoryXY" style="height: 400px;"></div>
            </div>
        </div>
        
        <div class="charts-container">
            <div class="chart-panel">
                <h3>足先速度</h3>
                <div id="velocityChart" style="height: 300px;"></div>
            </div>
            <div class="chart-panel">
                <h3>歩行メトリクス</h3>
                <div id="metricsChart" style="height: 300px;"></div>
            </div>
        </div>
        
        <div class="leg-status">
            <div class="leg-card">
                <h4 style="color: #dc3545;">🦵 RF (右前脚)</h4>
                <div id="rf-position" class="leg-position">位置: [0, 0, 0]</div>
                <div id="rf-velocity" class="leg-position">速度: 0 mm/s</div>
            </div>
            <div class="leg-card">
                <h4 style="color: #007bff;">🦵 LF (左前脚)</h4>
                <div id="lf-position" class="leg-position">位置: [0, 0, 0]</div>
                <div id="lf-velocity" class="leg-position">速度: 0 mm/s</div>
            </div>
            <div class="leg-card">
                <h4 style="color: #28a745;">🦵 LM (左中脚)</h4>
                <div id="lm-position" class="leg-position">位置: [0, 0, 0]</div>
                <div id="lm-velocity" class="leg-position">速度: 0 mm/s</div>
            </div>
            <div class="leg-card">
                <h4 style="color: #ffc107;">🦵 LB (左後脚)</h4>
                <div id="lb-position" class="leg-position">位置: [0, 0, 0]</div>
                <div id="lb-velocity" class="leg-position">速度: 0 mm/s</div>
            </div>
            <div class="leg-card">
                <h4 style="color: #6f42c1;">🦵 RB (右後脚)</h4>
                <div id="rb-position" class="leg-position">位置: [0, 0, 0]</div>
                <div id="rb-velocity" class="leg-position">速度: 0 mm/s</div>
            </div>
            <div class="leg-card">
                <h4 style="color: #fd7e14;">🦵 RM (右中脚)</h4>
                <div id="rm-position" class="leg-position">位置: [0, 0, 0]</div>
                <div id="rm-velocity" class="leg-position">速度: 0 mm/s</div>
            </div>
        </div>
    </div>

    <script>
        // Socket.IO接続
        const socket = io();
        
        // 接続状態の管理
        socket.on('connect', function() {
            document.getElementById('connectionStatus').className = 'connection-status connected';
            document.getElementById('connectionStatus').innerHTML = '🟢 接続済み';
        });
        
        socket.on('disconnect', function() {
            document.getElementById('connectionStatus').className = 'connection-status disconnected';
            document.getElementById('connectionStatus').innerHTML = '🔴 切断';
        });
        
        // データ更新の処理
        socket.on('data_update', function(data) {
            updateStatusPanel(data.system_status);
            updateLegPositions(data.positions, data.velocities);
            updateCharts(data.trajectories, data.velocities);
            updateMetrics(data.metrics);
        });
        
        function updateStatusPanel(status) {
            document.getElementById('walkingStatus').textContent = 
                status.is_walking ? '歩行中' : '待機中';
            document.getElementById('walkingStatus').className = 
                'status-value ' + (status.is_walking ? 'status-good' : 'status-warning');
            
            document.getElementById('coordinationScore').textContent = 
                Math.round(status.coordination_score) + '%';
            document.getElementById('coordinationScore').className = 
                'status-value ' + getScoreClass(status.coordination_score);
            
            document.getElementById('stabilityMargin').textContent = 
                Math.round(status.stability_margin * 10) / 10 + 'mm';
            document.getElementById('stabilityMargin').className = 
                'status-value ' + (status.stability_margin > 10 ? 'status-good' : 
                                 status.stability_margin > 5 ? 'status-warning' : 'status-danger');
            
            document.getElementById('uptime').textContent = 
                Math.round(status.uptime) + 's';
        }
        
        function updateLegPositions(positions, velocities) {
            const legIds = ['RF', 'LF', 'LM', 'LB', 'RB', 'RM'];
            
            legIds.forEach(legId => {
                const pos = positions[legId] || [0, 0, 0];
                const vel = velocities[legId] || 0;
                
                const legIdLower = legId.toLowerCase();
                const posElement = document.getElementById(`${legIdLower}-position`);
                const velElement = document.getElementById(`${legIdLower}-velocity`);
                
                if (posElement) {
                    posElement.textContent = 
                        `位置: [${pos[0].toFixed(1)}, ${pos[1].toFixed(1)}, ${pos[2].toFixed(1)}]`;
                }
                
                if (velElement) {
                    velElement.textContent = `速度: ${vel.toFixed(1)} mm/s`;
                    velElement.className = 'leg-position ' + getVelocityClass(vel);
                }
            });
        }
        
        function updateCharts(trajectories, velocities) {
            // 3D軌道チャート
            const traces3d = [];
            const legColors = {
                'RF': '#dc3545', 'LF': '#007bff', 'LM': '#28a745',
                'LB': '#ffc107', 'RB': '#6f42c1', 'RM': '#fd7e14'
            };
            
            Object.keys(trajectories).forEach(legId => {
                const trajectory = trajectories[legId];
                if (trajectory.length > 0) {
                    const x = trajectory.map(p => p[0]);
                    const y = trajectory.map(p => p[1]);
                    const z = trajectory.map(p => p[2]);
                    
                    traces3d.push({
                        x: x, y: y, z: z,
                        type: 'scatter3d',
                        mode: 'lines+markers',
                        name: legId,
                        line: { color: legColors[legId], width: 3 },
                        marker: { size: 3 }
                    });
                }
            });
            
            const layout3d = {
                title: '3D足先軌道',
                scene: {
                    xaxis: { title: 'X (mm)', range: [50, 250] },
                    yaxis: { title: 'Y (mm)', range: [-100, 100] },
                    zaxis: { title: 'Z (mm)', range: [-150, -50] }
                },
                margin: { l: 0, r: 0, b: 0, t: 30 }
            };
            
            Plotly.newPlot('trajectory3d', traces3d, layout3d);
            
            // XY平面軌道チャート
            const tracesXY = [];
            Object.keys(trajectories).forEach(legId => {
                const trajectory = trajectories[legId];
                if (trajectory.length > 0) {
                    const x = trajectory.map(p => p[0]);
                    const y = trajectory.map(p => p[1]);
                    
                    tracesXY.push({
                        x: x, y: y,
                        type: 'scatter',
                        mode: 'lines+markers',
                        name: legId,
                        line: { color: legColors[legId], width: 3 },
                        marker: { size: 4 }
                    });
                }
            });
            
            const layoutXY = {
                title: 'XY平面軌道',
                xaxis: { title: 'X (mm)', range: [50, 250] },
                yaxis: { title: 'Y (mm)', range: [-100, 100] },
                margin: { l: 40, r: 10, b: 40, t: 30 }
            };
            
            Plotly.newPlot('trajectoryXY', tracesXY, layoutXY);
            
            // 速度チャート
            const velocityTraces = [];
            Object.keys(velocities).forEach(legId => {
                velocityTraces.push({
                    y: [velocities[legId]],
                    type: 'bar',
                    name: legId,
                    marker: { color: legColors[legId] }
                });
            });
            
            const velocityLayout = {
                title: '現在の足先速度',
                yaxis: { title: '速度 (mm/s)' },
                margin: { l: 40, r: 10, b: 40, t: 30 }
            };
            
            Plotly.newPlot('velocityChart', velocityTraces, velocityLayout);
        }
        
        function updateMetrics(metrics) {
            if (!metrics || !metrics.metrics) return;
            
            const m = metrics.metrics;
            const traces = [{
                x: ['協調性', '安定性', '規則性', '効率', '一貫性'],
                y: [
                    m.coordination_score || 0,
                    (m.stability_margin || 0) * 10, // mm → %変換
                    m.step_regularity || 0,
                    m.gait_efficiency || 0,
                    m.phase_consistency || 0
                ],
                type: 'bar',
                marker: {
                    color: ['#007bff', '#28a745', '#ffc107', '#dc3545', '#6f42c1']
                }
            }];
            
            const layout = {
                title: '歩行メトリクス',
                yaxis: { title: 'スコア (%)', range: [0, 100] },
                margin: { l: 40, r: 10, b: 40, t: 30 }
            };
            
            Plotly.newPlot('metricsChart', traces, layout);
        }
        
        function getScoreClass(score) {
            if (score >= 70) return 'status-good';
            if (score >= 50) return 'status-warning';
            return 'status-danger';
        }
        
        function getVelocityClass(velocity) {
            if (velocity > 150) return 'status-danger';
            if (velocity > 100) return 'status-warning';
            return 'status-good';
        }
        
        // 初期チャートの設定
        document.addEventListener('DOMContentLoaded', function() {
            // 空のチャートで初期化
            Plotly.newPlot('trajectory3d', [], {
                title: '3D足先軌道',
                scene: {
                    xaxis: { title: 'X (mm)' },
                    yaxis: { title: 'Y (mm)' },
                    zaxis: { title: 'Z (mm)' }
                }
            });
            
            Plotly.newPlot('trajectoryXY', [], {
                title: 'XY平面軌道',
                xaxis: { title: 'X (mm)' },
                yaxis: { title: 'Y (mm)' }
            });
            
            Plotly.newPlot('velocityChart', [], {
                title: '足先速度',
                yaxis: { title: '速度 (mm/s)' }
            });
            
            Plotly.newPlot('metricsChart', [], {
                title: '歩行メトリクス',
                yaxis: { title: 'スコア (%)' }
            });
        });
    </script>
</body>
</html>
"""
    
    # テンプレートディレクトリを作成
    import os
    template_dir = os.path.join(os.path.dirname(__file__), 'templates')
    os.makedirs(template_dir, exist_ok=True)
    
    # HTMLファイルを保存
    with open(os.path.join(template_dir, 'hexapod_monitor.html'), 'w', encoding='utf-8') as f:
        f.write(html_content)

if __name__ == '__main__':
    try:
        # HTMLテンプレートの作成
        create_html_template()
        
        # サーバーの起動
        server = WebMonitorServer()
        server.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Web Monitor Server shutting down")
    except Exception as e:
        rospy.logerr(f"Error in Web Monitor Server: {e}")
        import traceback
        traceback.print_exc()
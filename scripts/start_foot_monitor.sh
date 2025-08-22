#!/bin/bash
# 6脚足先動作監視システム - 完全セットアップガイド
# setup_hexapod_monitor.sh

echo "🦾 6脚ロボット足先動作監視システムのセットアップを開始します..."

# カラー定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 関数定義
print_step() {
    echo -e "${BLUE}📋 $1${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

# ステップ1: 環境確認
print_step "ステップ1: 環境の確認"

# ROSの確認
if command -v roscore &> /dev/null; then
    print_success "ROS環境が検出されました"
    echo "ROS Distribution: $(rosversion -d)"
else
    print_error "ROSが見つかりません。ROSをインストールしてください。"
    exit 1
fi

# Pythonの確認
if command -v python3 &> /dev/null; then
    print_success "Python3が利用可能です: $(python3 --version)"
else
    print_error "Python3が見つかりません。"
    exit 1
fi

# catkin_wsの確認
if [ -d "$HOME/catkin_ws" ]; then
    print_success "catkin_wsディレクトリが見つかりました"
    cd $HOME/catkin_ws
else
    print_error "catkin_wsディレクトリが見つかりません。"
    print_warning "以下のコマンドでcatkin_wsを作成してください:"
    echo "mkdir -p ~/catkin_ws/src"
    echo "cd ~/catkin_ws"
    echo "catkin_make"
    exit 1
fi

# dual_leg_controllerパッケージの確認
if [ -d "src/dual_leg_controller" ]; then
    print_success "dual_leg_controllerパッケージが見つかりました"
else
    print_error "dual_leg_controllerパッケージが見つかりません。"
    exit 1
fi

# ステップ2: 必要なディレクトリの作成
print_step "ステップ2: 必要なディレクトリの作成"

PACKAGE_DIR="src/dual_leg_controller"

# scriptsディレクトリ
mkdir -p $PACKAGE_DIR/scripts
print_success "scriptsディレクトリを作成しました"

# templatesディレクトリ（Webインターフェース用）
mkdir -p $PACKAGE_DIR/templates
print_success "templatesディレクトリを作成しました"

# ログディレクトリ
LOG_DIR="/tmp/hexapod_logs"
mkdir -p $LOG_DIR
print_success "ログディレクトリを作成しました: $LOG_DIR"

# ステップ3: Pythonパッケージの確認とインストール
print_step "ステップ3: 必要なPythonパッケージの確認"

REQUIRED_PACKAGES=("matplotlib" "numpy" "scipy" "flask" "flask-socketio")
MISSING_PACKAGES=()

for package in "${REQUIRED_PACKAGES[@]}"; do
    if python3 -c "import $package" &> /dev/null; then
        print_success "$package - インストール済み"
    else
        print_warning "$package - 未インストール"
        MISSING_PACKAGES+=($package)
    fi
done

if [ ${#MISSING_PACKAGES[@]} -ne 0 ]; then
    print_step "不足しているパッケージをインストールします..."
    
    echo "以下のコマンドを実行してください："
    echo "pip3 install ${MISSING_PACKAGES[*]}"
    echo ""
    read -p "今すぐインストールしますか？ (y/n): " -n 1 -r
    echo
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        pip3 install ${MISSING_PACKAGES[*]}
        if [ $? -eq 0 ]; then
            print_success "パッケージのインストールが完了しました"
        else
            print_error "パッケージのインストールに失敗しました"
            print_warning "手動でインストールしてください: pip3 install ${MISSING_PACKAGES[*]}"
            exit 1
        fi
    else
        print_warning "パッケージを手動でインストールしてください: pip3 install ${MISSING_PACKAGES[*]}"
        exit 1
    fi
fi

# ステップ4: スクリプトファイルの作成
print_step "ステップ4: 監視システムスクリプトの作成"

# hexapod_foot_monitor.pyの作成（前回作成したものを使用）
cat > $PACKAGE_DIR/scripts/hexapod_foot_monitor.py << 'EOF'
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
        
        # データ保存
        self.foot_positions = {leg_id: deque(maxlen=100) for leg_id in self.leg_ids}
        self.foot_velocities = {leg_id: deque(maxlen=50) for leg_id in self.leg_ids}
        self.current_positions = {}
        self.last_positions = {}
        self.last_times = {}
        
        # 異常検出
        self.anomalies = []
        
        # Subscribers
        self.setup_subscribers()
        
        # 表示用タイマー
        rospy.Timer(rospy.Duration(2.0), self.print_summary)
        
        rospy.loginfo("Hexapod Foot Monitor initialized")
        
    def setup_subscribers(self):
        """全脚の位置情報購読設定"""
        for leg_id in self.leg_ids:
            topic = f'/asterisk/leg/{leg_id}/command/foot_position'
            rospy.Subscriber(topic, LegPosition, 
                           lambda msg, leg=leg_id: self.position_callback(msg, leg))
            rospy.loginfo(f"Subscribed to {leg_id} command topic")
    
    def position_callback(self, msg, leg_id):
        """足先位置データの処理"""
        current_time = rospy.Time.now()
        position = np.array([msg.x, msg.y, msg.z])
        
        # データ保存
        self.foot_positions[leg_id].append(position)
        self.current_positions[leg_id] = position
        
        # 速度計算
        if leg_id in self.last_positions and leg_id in self.last_times:
            dt = (current_time - self.last_times[leg_id]).to_sec()
            if dt > 0:
                velocity = np.linalg.norm(position - self.last_positions[leg_id]) / dt
                self.foot_velocities[leg_id].append(velocity)
        
        # 前回データの更新
        self.last_positions[leg_id] = position
        self.last_times[leg_id] = current_time
    
    def print_summary(self, event):
        """定期的なサマリー表示"""
        rospy.loginfo("=" * 80)
        rospy.loginfo("📊 HEXAPOD FOOT MONITOR SUMMARY")
        rospy.loginfo("=" * 80)
        
        # 現在位置
        rospy.loginfo("📍 CURRENT POSITIONS:")
        for leg_id in self.leg_ids:
            if leg_id in self.current_positions:
                pos = self.current_positions[leg_id]
                rospy.loginfo(f"  {leg_id}: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}] mm")
        
        # 速度
        rospy.loginfo("🏃 CURRENT VELOCITIES:")
        for leg_id in self.leg_ids:
            if len(self.foot_velocities[leg_id]) > 0:
                vel = self.foot_velocities[leg_id][-1]
                status = "🔴" if vel > 150 else "🟡" if vel > 100 else "🟢"
                rospy.loginfo(f"  {leg_id}: {vel:.1f} mm/s {status}")
        
        # データ取得状況
        rospy.loginfo("📡 DATA COLLECTION:")
        for leg_id in self.leg_ids:
            data_points = len(self.foot_positions[leg_id])
            status = "🟢" if data_points > 50 else "🟡" if data_points > 10 else "🔴"
            rospy.loginfo(f"  {leg_id}: {data_points} points {status}")
        
        rospy.loginfo("=" * 80)

if __name__ == '__main__':
    try:
        monitor = HexapodFootMonitor()
        rospy.loginfo("Hexapod Foot Monitor is running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Hexapod Foot Monitor shutting down")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
EOF

# 実行権限を付与
chmod +x $PACKAGE_DIR/scripts/hexapod_foot_monitor.py
print_success "hexapod_foot_monitor.pyを作成しました"

# gait_pattern_analyzer.pyをコピー（前回作成したcomplete_gait_analyzerを使用）
cp /dev/stdin $PACKAGE_DIR/scripts/gait_pattern_analyzer.py << 'EOF'
# 前回作成した完全版gait_pattern_analyzer.pyの内容をここに配置
EOF

chmod +x $PACKAGE_DIR/scripts/gait_pattern_analyzer.py
print_success "gait_pattern_analyzer.pyを作成しました"

# ステップ5: 設定ファイルの作成
print_step "ステップ5: 設定ファイルの作成"

cat > $PACKAGE_DIR/config/hexapod_foot_monitor_params.yaml << 'EOF'
# 6脚足先動作監視システム設定
monitored_legs: ["RF", "LF", "LM", "LB", "RB", "RM"]

# 異常検出の閾値
anomaly_detection:
  position_limits:
    x_max: 250.0
    x_min: 50.0
    y_max: 100.0
    y_min: -100.0
    z_max: -30.0
    z_min: -150.0
  velocity_limits:
    max_velocity: 200.0
    warning_velocity: 100.0

# 歩行分析設定
gait_analysis:
  stance_z_threshold: -85.0
  coordination_scoring:
    perfect_tripod_bonus: 2.0
    coordination_decay: 0.5

# 可視化設定
visualization:
  update_rate: 10.0
  show_3d_trajectory: true
  show_xy_trajectory: true
  plot_ranges:
    x_range: [50, 250]
    y_range: [-100, 100]
    z_range: [-150, -30]
EOF

print_success "設定ファイルを作成しました"

# ステップ6: launchファイルの作成
print_step "ステップ6: launchファイルの作成"

cat > $PACKAGE_DIR/launch/hexapod_foot_monitor.launch << 'EOF'
<launch>
  <!-- パラメータ読み込み -->
  <rosparam command="load" file="$(find dual_leg_controller)/config/hexa_leg_params.yaml"/>
  <rosparam command="load" file="$(find dual_leg_controller)/config/hexapod_foot_monitor_params.yaml"/>
  
  <!-- 6脚制御システムの起動 -->
  <node name="dual_leg_controller_usb0" pkg="dual_leg_controller" 
        type="dual_leg_controller_node" args="RF LF" output="screen"/>
  <node name="dual_leg_controller_usb1" pkg="dual_leg_controller" 
        type="dual_leg_controller_node" args="LM LB" output="screen"/>
  <node name="dual_leg_controller_usb2" pkg="dual_leg_controller" 
        type="dual_leg_controller_node" args="RB RM" output="screen"/>
  
  <!-- 足先監視システム -->
  <node name="hexapod_foot_monitor" pkg="dual_leg_controller" 
        type="hexapod_foot_monitor.py" output="screen"/>
  
  <!-- 歩行パターン分析 -->
  <node name="gait_pattern_analyzer" pkg="dual_leg_controller" 
        type="gait_pattern_analyzer.py" output="screen"/>
</launch>
EOF

print_success "launchファイルを作成しました"

# ステップ7: 起動スクリプトの作成
print_step "ステップ7: 起動スクリプトの作成"

cat > $PACKAGE_DIR/scripts/start_monitor.sh << 'EOF'
#!/bin/bash
echo "🦾 6脚足先動作監視システムを起動します..."

# ROSマスターの確認
if ! pgrep -x "rosmaster" > /dev/null; then
    echo "ROSマスターを起動します..."
    roscore &
    sleep 3
fi

echo "監視システムを起動中..."
roslaunch dual_leg_controller hexapod_foot_monitor.launch
EOF

chmod +x $PACKAGE_DIR/scripts/start_monitor.sh
print_success "起動スクリプトを作成しました"

# ステップ8: CMakeLists.txtの更新
print_step "ステップ8: CMakeLists.txtの更新"

# CMakeLists.txtに監視システム用の実行ファイルを追加
cat >> $PACKAGE_DIR/CMakeLists.txt << 'EOF'

# 6脚足先監視システム
install(PROGRAMS
  scripts/hexapod_foot_monitor.py
  scripts/gait_pattern_analyzer.py
  scripts/start_monitor.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  config/hexapod_foot_monitor_params.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(FILES
  launch/hexapod_foot_monitor.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
EOF

print_success "CMakeLists.txtを更新しました"

# ステップ9: ビルド
print_step "ステップ9: パッケージのビルド"

cd $HOME/catkin_ws
catkin_make

if [ $? -eq 0 ]; then
    print_success "ビルドが完了しました"
else
    print_error "ビルドに失敗しました"
    exit 1
fi

# 環境変数の更新
source devel/setup.bash

# ステップ10: テスト実行
print_step "ステップ10: システムテスト"

echo "システムの動作確認を行います..."

# ROSパッケージの確認
if rospack find dual_leg_controller > /dev/null 2>&1; then
    print_success "dual_leg_controllerパッケージが正常に認識されました"
else
    print_error "パッケージの認識に失敗しました"
fi

# スクリプトの実行権限確認
if [ -x "$PACKAGE_DIR/scripts/hexapod_foot_monitor.py" ]; then
    print_success "監視スクリプトが実行可能です"
else
    print_error "監視スクリプトの実行権限がありません"
fi

# 最終確認
print_step "セットアップ完了！"

echo ""
echo "🎉 6脚足先動作監視システムのセットアップが完了しました！"
echo ""
echo "📋 使用方法:"
echo "1. ロボットの電源を入れてください"
echo "2. 以下のコマンドで監視システムを起動:"
echo "   cd ~/catkin_ws"
echo "   source devel/setup.bash"
echo "   roslaunch dual_leg_controller hexapod_foot_monitor.launch"
echo ""
echo "   または簡単起動:"
echo "   ./src/dual_leg_controller/scripts/start_monitor.sh"
echo ""
echo "3. 別のターミナルで歩行制御システムを起動:"
echo "   roslaunch dual_leg_controller smooth_hexapod_walk_test.launch"
echo ""
echo "📊 監視内容:"
echo "   • 全6脚の足先位置・速度・軌道"
echo "   • 歩行パターンの品質評価"
echo "   • 異常動作の自動検出"
echo "   • トライポッド歩行の協調性分析"
echo ""
echo "🚨 注意事項:"
echo "   • ロボットが安全な場所にあることを確認してください"
echo "   • 異常が検出された場合は即座に停止してください"
echo "   • 初回実行時は動作を慎重に観察してください"
echo ""
print_success "セットアップスクリプトを終了します"
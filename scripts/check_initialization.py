#!/usr/bin/env python3
"""
システム初期化チェッカー
dual_leg_controllerが正常に起動しているかチェックして
安全にテストを開始できる状態かを確認する
"""

import rospy
import time
from dual_leg_controller.msg import LegPosition, LegCommand
from std_msgs.msg import String

class InitializationChecker:
    def __init__(self):
        rospy.init_node('initialization_checker', anonymous=True)
        
        self.wait_time = rospy.get_param('~wait_time', 3.0)
        self.target_leg = rospy.get_param('~target_leg', 'RF')
        
        # チェック状態
        self.controller_ready = False
        self.position_received = False
        self.state_received = False
        
        rospy.loginfo("Checking system initialization...")
        rospy.loginfo(f"Waiting {self.wait_time} seconds for system to stabilize...")
        
        # 初期待機
        time.sleep(self.wait_time)
        
        # チェック開始
        self.check_system()
    
    def check_system(self):
        """システムの初期化状態をチェック"""
        rospy.loginfo("=== SYSTEM INITIALIZATION CHECK ===")
        
        # 1. dual_leg_controllerノードの存在確認
        self.check_controller_node()
        
        # 2. トピックの存在確認
        self.check_topics()
        
        # 3. メッセージの受信確認
        self.check_message_flow()
        
        # 4. 総合判定
        self.print_final_status()
    
    def check_controller_node(self):
        """dual_leg_controllerノードが起動しているかチェック"""
        try:
            nodes = rospy.get_published_topics()
            controller_topics = [topic for topic, msg_type in nodes 
                               if f'/asterisk/leg/{self.target_leg}' in topic]
            
            if len(controller_topics) > 0:
                rospy.loginfo(f"✓ Controller node appears to be running ({len(controller_topics)} topics found)")
                self.controller_ready = True
            else:
                rospy.logwarn(f"✗ Controller node may not be running (no topics found for {self.target_leg})")
                
        except Exception as e:
            rospy.logerr(f"✗ Error checking controller node: {e}")
    
    def check_topics(self):
        """必要なトピックが存在するかチェック"""
        required_topics = [
            f'/asterisk/leg/{self.target_leg}/command/foot_position',
            f'/asterisk/leg/{self.target_leg}/command/joint_angles',
            f'/asterisk/leg/{self.target_leg}/state/foot_position',
            f'/asterisk/leg/{self.target_leg}/state/read_angle'
        ]
        
        published_topics = [topic for topic, msg_type in rospy.get_published_topics()]
        
        for topic in required_topics:
            if topic in published_topics:
                rospy.loginfo(f"✓ Topic found: {topic}")
            else:
                rospy.logwarn(f"✗ Topic missing: {topic}")
    
    def check_message_flow(self):
        """メッセージが正常に流れているかチェック"""
        rospy.loginfo("Checking message flow...")
        
        # 位置状態の受信チェック
        try:
            rospy.loginfo("Waiting for position state message...")
            msg = rospy.wait_for_message(f'/asterisk/leg/{self.target_leg}/state/foot_position', 
                                       LegPosition, timeout=5.0)
            rospy.loginfo(f"✓ Position state received: [{msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}]")
            self.position_received = True
            
            # 位置の妥当性チェック
            if self.is_reasonable_position(msg.x, msg.y, msg.z):
                rospy.loginfo("✓ Position values appear reasonable")
            else:
                rospy.logwarn(f"⚠ Position values may be unusual: [{msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}]")
                
        except rospy.ROSException:
            rospy.logwarn("✗ Timeout waiting for position state message")
        
        # 関節角度の受信チェック
        try:
            rospy.loginfo("Waiting for joint angle message...")
            msg = rospy.wait_for_message(f'/asterisk/leg/{self.target_leg}/state/read_angle', 
                                       LegCommand, timeout=5.0)
            rospy.loginfo(f"✓ Joint angles received: [{msg.coxa_angle:.2f}, {msg.femur_angle:.2f}, {msg.tibia_angle:.2f}]")
            self.state_received = True
            
        except rospy.ROSException:
            rospy.logwarn("✗ Timeout waiting for joint angle message")
    
    def is_reasonable_position(self, x, y, z):
        """位置が合理的な範囲内かチェック"""
        # 大まかな合理性チェック（具体的な範囲は調整可能）
        return (50.0 <= x <= 250.0 and 
               -100.0 <= y <= 100.0 and 
               -150.0 <= z <= -30.0)
    
    def print_final_status(self):
        """最終ステータスの表示"""
        rospy.loginfo("=== INITIALIZATION CHECK RESULTS ===")
        
        all_good = (self.controller_ready and 
                   self.position_received and 
                   self.state_received)
        
        if all_good:
            rospy.loginfo("🟢 SYSTEM READY - All checks passed!")
            rospy.loginfo("It should be safe to start the walking test.")
            rospy.loginfo("=" * 50)
            rospy.loginfo("SAFETY REMINDERS:")
            rospy.loginfo("1. Keep your hand near the emergency stop")
            rospy.loginfo("2. Start with small movements")
            rospy.loginfo("3. Watch for any unusual sounds or movements")
            rospy.loginfo("4. Use 'h' key to return home at any time")
            rospy.loginfo("=" * 50)
        else:
            rospy.logwarn("🟡 SYSTEM NOT FULLY READY")
            rospy.logwarn("Some checks failed. Please review the issues above.")
            rospy.logwarn("It may not be safe to start the walking test yet.")
        
        # 具体的なアドバイス
        if not self.controller_ready:
            rospy.logwarn("→ Check if dual_leg_controller is running properly")
        if not self.position_received:
            rospy.logwarn("→ Check Dynamixel connections and power")
        if not self.state_received:
            rospy.logwarn("→ Check if motors are responding to read commands")

if __name__ == '__main__':
    try:
        checker = InitializationChecker()
        rospy.loginfo("Initialization check completed. You may now proceed with the test.")
    except rospy.ROSInterruptException:
        rospy.loginfo("Initialization check interrupted")
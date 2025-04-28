#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include "dual_leg_controller/LegPosition.h"

/**
 * 手動トライポッドスタンスプログラム
 * 
 * このプログラムは、六脚ロボットの脚を2つのグループに分け、
 * 交互に上下させる基本的な動作を実現します。
 * 各脚グループの切り替えは一定時間ごとに自動的に行われます。
 */
class ManualTripodStance {
public:
    ManualTripodStance(ros::NodeHandle& nh) : nh_(nh) {
        // パラメータの読み込み
        initializeParameters();
        
        // パブリッシャーの設定
        initializePublishers();
        
        // タイマーの初期化
        timer_ = nh_.createTimer(ros::Duration(update_interval_), 
                                &ManualTripodStance::timerCallback, this);
        
        ROS_INFO("Manual Tripod Stance initialized");
    }
    
    // パラメータの初期化
    void initializeParameters() {
        // 脚のグループ化: トライポッド歩行の基本配置
        // グループA: 右前、左中、右後
        leg_groups_["A"].push_back("RF");
        leg_groups_["A"].push_back("LM");
        leg_groups_["A"].push_back("RB");
        
        // グループB: 左前、右中、左後
        leg_groups_["B"].push_back("LF");
        leg_groups_["B"].push_back("RM");
        leg_groups_["B"].push_back("LB");
        
        // デフォルトの脚位置
        default_height_ = 0.0;     // デフォルトの高さ (地面にある状態) [mm]
        raised_height_ = -30.0;    // 上げた時の高さ [mm]
        leg_x_ = 150.0;            // 脚先のX座標 (前方向) [mm]
        leg_y_ = 150.0;            // 脚先のY座標 (横方向) [mm]
        
        // 更新間隔とタイミングパラメータ
        update_interval_ = 0.02;   // 制御更新間隔 [s]
        toggle_interval_ = 1.0;    // 脚グループ切り替え間隔 [s]
        
        // カスタムパラメータがある場合はROSパラメータから読み込む
        nh_.param<double>("default_height", default_height_, default_height_);
        nh_.param<double>("raised_height", raised_height_, raised_height_);
        nh_.param<double>("leg_x", leg_x_, leg_x_);
        nh_.param<double>("leg_y", leg_y_, leg_y_);
        nh_.param<double>("update_interval", update_interval_, update_interval_);
        nh_.param<double>("toggle_interval", toggle_interval_, toggle_interval_);
        
        // 状態の初期化
        current_group_ = "A";      // 最初に上げるグループ
        last_toggle_time_ = ros::Time::now();
    }
    
    // パブリッシャーの初期化
    void initializePublishers() {
        // 各脚の位置指令パブリッシャー
        std::vector<std::string> all_legs {"RF", "LF", "RM", "LM", "RB", "LB"};
        
        for (const auto& leg_id : all_legs) {
            std::string topic = "/asterisk/leg/" + leg_id + "/command/foot_position";
            leg_pubs_[leg_id] = nh_.advertise<dual_leg_controller::LegPosition>(topic, 1);
            
            ROS_INFO("Publisher created for leg %s", leg_id.c_str());
        }
    }
    
    // 脚の位置を設定
    void setLegPosition(const std::string& leg_id, double x, double y, double z) {
        if (leg_pubs_.find(leg_id) == leg_pubs_.end()) {
            ROS_ERROR("Unknown leg ID: %s", leg_id.c_str());
            return;
        }
        
        dual_leg_controller::LegPosition msg;
        msg.x = x;
        msg.y = y;
        msg.z = z;
        
        leg_pubs_[leg_id].publish(msg);
    }
    
    // トライポッドスタンスの更新
    void updateTripodStance() {
        ros::Time now = ros::Time::now();
        
        // グループ切り替えのタイミングを確認
        if ((now - last_toggle_time_).toSec() > toggle_interval_) {
            // グループを切り替え
            current_group_ = (current_group_ == "A") ? "B" : "A";
            last_toggle_time_ = now;
            
            ROS_INFO("Switched to leg group %s", current_group_.c_str());
        }
        
        // 各脚グループの位置設定
        for (const auto& group : leg_groups_) {
            const std::string& group_id = group.first;
            const std::vector<std::string>& legs = group.second;
            
            // 現在アクティブなグループは上げる、それ以外は下げる
            double z = (group_id == current_group_) ? raised_height_ : default_height_;
            
            for (const auto& leg_id : legs) {
                // 脚の左右配置に応じてY座標を調整
                double y = leg_y_;
                if (leg_id.at(0) == 'L') { // 左脚の場合
                    y = -leg_y_;
                }
                
                // 脚の前後位置に応じてX座標を調整
                double x = leg_x_;
                if (leg_id.at(1) == 'M') { // 中脚の場合
                    x = 0;
                } else if (leg_id.at(1) == 'B') { // 後脚の場合
                    x = -leg_x_;
                }
                
                setLegPosition(leg_id, x, y, z);
            }
        }
    }
    
    // タイマーコールバック
    void timerCallback(const ros::TimerEvent&) {
        updateTripodStance();
    }
    
private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    
    // 脚グループの管理
    std::map<std::string, std::vector<std::string>> leg_groups_;
    std::map<std::string, ros::Publisher> leg_pubs_;
    
    // 脚位置パラメータ
    double default_height_;  // デフォルトの高さ
    double raised_height_;   // 上げた時の高さ
    double leg_x_;           // 前後位置
    double leg_y_;           // 左右位置
    
    // タイミングパラメータ
    double update_interval_; // 制御更新間隔
    double toggle_interval_; // 脚グループ切り替え間隔
    
    // 状態管理
    std::string current_group_;  // 現在アクティブなグループ
    ros::Time last_toggle_time_; // 最後にグループを切り替えた時間
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "manual_tripod_stance");
    ros::NodeHandle nh;
    
    ManualTripodStance tripod_stance(nh);
    
    ROS_INFO("Manual Tripod Stance node started");
    ros::spin();
    
    return 0;
}
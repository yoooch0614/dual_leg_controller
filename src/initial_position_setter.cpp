#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include "dual_leg_controller/LegPosition.h"

/**
 * 初期位置設定プログラム
 * 
 * このプログラムは、六脚ロボットの全ての脚を初期位置に移動させます。
 * 初期位置は、各脚がデフォルトの高さで、適切な前後左右の位置に配置されます。
 */
class InitialPositionSetter {
public:
    InitialPositionSetter(ros::NodeHandle& nh) : nh_(nh) {
        // パラメータの読み込み
        initializeParameters();
        
        // パブリッシャーの設定
        initializePublishers();
        
        // タイマーの初期化（一度だけ実行する）
        timer_ = nh_.createTimer(ros::Duration(1.0), 
                                &InitialPositionSetter::timerCallback, this, true);
        
        ROS_INFO("Initial Position Setter initialized");
    }
    
    // パラメータの初期化
    void initializeParameters() {
        // 全ての脚のリスト
        all_legs_ = {"RF", "LF", "RM", "LM", "RB", "LB"};
        
        // デフォルトの脚位置パラメータ
        default_height_ = 0.0;     // デフォルトの高さ (地面にある状態) [mm]
        leg_x_ = 150.0;            // 脚先のX座標 (前方向) [mm]
        leg_y_ = 150.0;            // 脚先のY座標 (横方向) [mm]
        
        // カスタムパラメータがある場合はROSパラメータから読み込む
        nh_.param<double>("default_height", default_height_, default_height_);
        nh_.param<double>("leg_x", leg_x_, leg_x_);
        nh_.param<double>("leg_y", leg_y_, leg_y_);
        
        ROS_INFO("Parameters loaded: default_height=%.2f, leg_x=%.2f, leg_y=%.2f", 
                default_height_, leg_x_, leg_y_);
    }
    
    // パブリッシャーの初期化
    void initializePublishers() {
        // 各脚の位置指令パブリッシャー
        for (const auto& leg_id : all_legs_) {
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
        ROS_INFO("Set leg %s position to (%.2f, %.2f, %.2f)", 
                leg_id.c_str(), x, y, z);
    }
    
    // 全ての脚を初期位置に移動
    void moveToInitialPosition() {
        ROS_INFO("Moving all legs to initial positions...");
        
        // 各脚の初期位置を設定
        for (const auto& leg_id : all_legs_) {
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
            
            // 全ての脚を同じ高さに設定
            setLegPosition(leg_id, x, y, default_height_);
        }
        
        ROS_INFO("All legs moved to initial positions");
    }
    
    // タイマーコールバック（一度だけ実行）
    void timerCallback(const ros::TimerEvent&) {
        moveToInitialPosition();
        
        // 少し待ってからノードを終了するタイマーを設定（5秒後）
        shutdown_timer_ = nh_.createTimer(ros::Duration(5.0), 
                                        &InitialPositionSetter::shutdownCallback, this, true);
    }
    
    // シャットダウンコールバック
    void shutdownCallback(const ros::TimerEvent&) {
        ROS_INFO("Initial position setting completed. Shutting down node.");
        ros::shutdown();
    }
    
private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Timer shutdown_timer_;
    
    // 脚のリストと管理
    std::vector<std::string> all_legs_;
    std::map<std::string, ros::Publisher> leg_pubs_;
    
    // 脚位置パラメータ
    double default_height_;  // デフォルトの高さ
    double leg_x_;           // 前後位置
    double leg_y_;           // 左右位置
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "initial_position_setter");
    ros::NodeHandle nh;
    
    InitialPositionSetter setter(nh);
    
    ROS_INFO("Initial Position Setter node started");
    ros::spin();
    
    return 0;
}
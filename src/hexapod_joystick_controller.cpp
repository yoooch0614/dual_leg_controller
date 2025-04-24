#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <cmath>

class HexapodJoystickController {
public:
    HexapodJoystickController() : nh_("~"), last_joy_time_(ros::Time::now()) {
        // パラメータ読み込み
        loadParameters();
        
        // パブリッシャー
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        enable_pub_ = nh_.advertise<std_msgs::Bool>("/hexapod/enable", 1);
        emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("/hexapod/emergency_stop", 1);
        home_pub_ = nh_.advertise<std_msgs::Bool>("/hexapod/home", 1);
        
        // サブスクライバー
        joy_sub_ = nh_.subscribe("/joy", 1, &HexapodJoystickController::joyCallback, this);
        
        // 制御タイマー（高頻度更新）
        control_timer_ = nh_.createTimer(
            ros::Duration(1.0 / update_rate_), 
            &HexapodJoystickController::controlLoop, this);
        
        // 初期化
        current_cmd_.linear.x = 0.0;
        current_cmd_.linear.y = 0.0;
        current_cmd_.angular.z = 0.0;
        
        target_cmd_.linear.x = 0.0;
        target_cmd_.linear.y = 0.0;
        target_cmd_.angular.z = 0.0;
        
        is_enabled_ = false;
        last_enable_button_ = false;
        last_emergency_button_ = false;
        last_home_button_ = false;
        
        ROS_INFO("Hexapod Joystick Controller initialized");
        printInstructions();
    }

private:
    ros::NodeHandle nh_;
    
    // Publishers
    ros::Publisher cmd_vel_pub_;
    ros::Publisher enable_pub_;
    ros::Publisher emergency_stop_pub_;
    ros::Publisher home_pub_;
    
    // Subscribers
    ros::Subscriber joy_sub_;
    
    // Timer
    ros::Timer control_timer_;
    
    // Parameters
    double deadzone_radius_;
    double max_linear_speed_;   // mm/s
    double max_angular_speed_;  // rad/s
    int enable_button_;
    int emergency_stop_button_;
    int home_button_;
    int linear_x_axis_;
    int linear_y_axis_;
    int angular_z_axis_;
    bool invert_linear_x_;
    bool invert_linear_y_;
    bool invert_angular_z_;
    double acceleration_limit_;  // mm/s²
    double angular_acceleration_limit_;  // rad/s²
    double update_rate_;
    
    // State variables
    geometry_msgs::Twist current_cmd_;
    geometry_msgs::Twist target_cmd_;
    bool is_enabled_;
    bool last_enable_button_;
    bool last_emergency_button_;
    bool last_home_button_;
    ros::Time last_joy_time_;
    sensor_msgs::Joy last_joy_msg_;
    
    void loadParameters() {
        nh_.param("deadzone_radius", deadzone_radius_, 0.1);
        nh_.param("max_linear_speed", max_linear_speed_, 80.0);
        nh_.param("max_angular_speed", max_angular_speed_, 1.0);
        nh_.param("enable_button", enable_button_, 4);
        nh_.param("emergency_stop_button", emergency_stop_button_, 5);
        nh_.param("home_button", home_button_, 8);
        nh_.param("linear_x_axis", linear_x_axis_, 1);
        nh_.param("linear_y_axis", linear_y_axis_, 0);
        nh_.param("angular_z_axis", angular_z_axis_, 3);
        nh_.param("invert_linear_x", invert_linear_x_, true);
        nh_.param("invert_linear_y", invert_linear_y_, false);
        nh_.param("invert_angular_z", invert_angular_z_, false);
        nh_.param("acceleration_limit", acceleration_limit_, 200.0);
        nh_.param("angular_acceleration_limit", angular_acceleration_limit_, 2.0);
        nh_.param("update_rate", update_rate_, 50.0);
        
        ROS_INFO("Joystick Controller Parameters:");
        ROS_INFO("  Deadzone radius: %.2f", deadzone_radius_);
        ROS_INFO("  Max linear speed: %.1f mm/s", max_linear_speed_);
        ROS_INFO("  Max angular speed: %.2f rad/s", max_angular_speed_);
        ROS_INFO("  Update rate: %.1f Hz", update_rate_);
    }
    
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
        last_joy_time_ = ros::Time::now();
        last_joy_msg_ = *msg;
        
        // ボタンの状態をチェック（エッジ検出）
        bool enable_button_pressed = false;
        bool emergency_button_pressed = false;
        bool home_button_pressed = false;
        
        if (msg->buttons.size() > std::max({enable_button_, emergency_stop_button_, home_button_})) {
            // Enable/Disable トグル
            if (msg->buttons[enable_button_] && !last_enable_button_) {
                enable_button_pressed = true;
            }
            
            // Emergency Stop
            if (msg->buttons[emergency_stop_button_] && !last_emergency_button_) {
                emergency_button_pressed = true;
            }
            
            // Home Position
            if (msg->buttons[home_button_] && !last_home_button_) {
                home_button_pressed = true;
            }
            
            last_enable_button_ = msg->buttons[enable_button_];
            last_emergency_button_ = msg->buttons[emergency_stop_button_];
            last_home_button_ = msg->buttons[home_button_];
        }
        
        // ボタンアクション実行
        if (enable_button_pressed) {
            is_enabled_ = !is_enabled_;
            std_msgs::Bool enable_msg;
            enable_msg.data = is_enabled_;
            enable_pub_.publish(enable_msg);
            ROS_INFO("Hexapod %s", is_enabled_ ? "ENABLED" : "DISABLED");
        }
        
        if (emergency_button_pressed) {
            is_enabled_ = false;
            std_msgs::Bool emergency_msg;
            emergency_msg.data = true;
            emergency_stop_pub_.publish(emergency_msg);
            
            // 速度を即座に0に
            current_cmd_.linear.x = 0.0;
            current_cmd_.linear.y = 0.0;
            current_cmd_.angular.z = 0.0;
            target_cmd_.linear.x = 0.0;
            target_cmd_.linear.y = 0.0;
            target_cmd_.angular.z = 0.0;
            
            ROS_WARN("EMERGENCY STOP ACTIVATED!");
        }
        
        if (home_button_pressed) {
            std_msgs::Bool home_msg;
            home_msg.data = true;
            home_pub_.publish(home_msg);
            ROS_INFO("Moving to home position");
        }
        
        // ジョイスティック軸の値を処理
        if (is_enabled_ && 
            msg->axes.size() > std::max({linear_x_axis_, linear_y_axis_, angular_z_axis_})) {
            
            double raw_x = msg->axes[linear_x_axis_];
            double raw_y = msg->axes[linear_y_axis_];
            double raw_angular = msg->axes[angular_z_axis_];
            
            // 軸の反転
            if (invert_linear_x_) raw_x = -raw_x;
            if (invert_linear_y_) raw_y = -raw_y;
            if (invert_angular_z_) raw_angular = -raw_angular;
            
            // デッドゾーン処理（直進用）
            double linear_magnitude = sqrt(raw_x * raw_x + raw_y * raw_y);
            if (linear_magnitude > deadzone_radius_) {
                // デッドゾーンを超えた分を正規化
                double scale = (linear_magnitude - deadzone_radius_) / (1.0 - deadzone_radius_);
                scale = std::min(1.0, scale);
                
                double normalized_x = (raw_x / linear_magnitude) * scale;
                double normalized_y = (raw_y / linear_magnitude) * scale;
                
                target_cmd_.linear.x = normalized_x * max_linear_speed_ / 1000.0;  // mm/s → m/s
                target_cmd_.linear.y = normalized_y * max_linear_speed_ / 1000.0;  // mm/s → m/s
            } else {
                target_cmd_.linear.x = 0.0;
                target_cmd_.linear.y = 0.0;
            }
            
            // 回転のデッドゾーン処理
            if (std::abs(raw_angular) > deadzone_radius_) {
                double angular_scale = (std::abs(raw_angular) - deadzone_radius_) / (1.0 - deadzone_radius_);
                angular_scale = std::min(1.0, angular_scale);
                
                target_cmd_.angular.z = (raw_angular > 0 ? angular_scale : -angular_scale) * max_angular_speed_;
            } else {
                target_cmd_.angular.z = 0.0;
            }
        } else if (!is_enabled_) {
            // 無効化時は目標値を0に
            target_cmd_.linear.x = 0.0;
            target_cmd_.linear.y = 0.0;
            target_cmd_.angular.z = 0.0;
        }
    }
    
    void controlLoop(const ros::TimerEvent& event) {
        // タイムアウトチェック（ジョイスティック接続確認）
        if ((ros::Time::now() - last_joy_time_).toSec() > 1.0) {
            // 1秒以上ジョイスティック入力がない場合は停止
            target_cmd_.linear.x = 0.0;
            target_cmd_.linear.y = 0.0;
            target_cmd_.angular.z = 0.0;
        }
        
        // スムーズな加速度制限
        double dt = 1.0 / update_rate_;
        
        // 線形速度の制限
        double linear_diff_x = target_cmd_.linear.x - current_cmd_.linear.x;
        double linear_diff_y = target_cmd_.linear.y - current_cmd_.linear.y;
        double max_linear_change = (acceleration_limit_ / 1000.0) * dt;  // mm/s² → m/s²
        
        if (std::abs(linear_diff_x) > max_linear_change) {
            current_cmd_.linear.x += (linear_diff_x > 0 ? max_linear_change : -max_linear_change);
        } else {
            current_cmd_.linear.x = target_cmd_.linear.x;
        }
        
        if (std::abs(linear_diff_y) > max_linear_change) {
            current_cmd_.linear.y += (linear_diff_y > 0 ? max_linear_change : -max_linear_change);
        } else {
            current_cmd_.linear.y = target_cmd_.linear.y;
        }
        
        // 角速度の制限
        double angular_diff = target_cmd_.angular.z - current_cmd_.angular.z;
        double max_angular_change = angular_acceleration_limit_ * dt;
        
        if (std::abs(angular_diff) > max_angular_change) {
            current_cmd_.angular.z += (angular_diff > 0 ? max_angular_change : -max_angular_change);
        } else {
            current_cmd_.angular.z = target_cmd_.angular.z;
        }
        
        // cmd_velの送信
        current_cmd_.linear.z = 0.0;  // 垂直移動は使用しない
        current_cmd_.angular.x = 0.0;
        current_cmd_.angular.y = 0.0;
        
        cmd_vel_pub_.publish(current_cmd_);
        
        // デバッグ情報の出力（1秒間隔）
        static ros::Time last_debug = ros::Time::now();
        if ((ros::Time::now() - last_debug).toSec() > 1.0) {
            if (is_enabled_ && (std::abs(current_cmd_.linear.x) > 0.001 || 
                               std::abs(current_cmd_.linear.y) > 0.001 || 
                               std::abs(current_cmd_.angular.z) > 0.001)) {
                ROS_INFO("Cmd: linear=(%.3f,%.3f)m/s, angular=%.3frad/s", 
                         current_cmd_.linear.x, current_cmd_.linear.y, current_cmd_.angular.z);
            }
            last_debug = ros::Time::now();
        }
    }
    
    void printInstructions() {
        ROS_INFO("=== HEXAPOD JOYSTICK CONTROLLER ===");
        ROS_INFO("6脚ロボット ジョイスティック制御システム");
        ROS_INFO("");
        ROS_INFO("CONTROLS:");
        ROS_INFO("  Left Stick: 移動制御 (前後・左右)");
        ROS_INFO("  Right Stick: 回転制御 (左右)");
        ROS_INFO("  L1/LB Button: Enable/Disable トグル");
        ROS_INFO("  R1/RB Button: 緊急停止");
        ROS_INFO("  Start Button: ホームポジション");
        ROS_INFO("");
        ROS_INFO("FEATURES:");
        ROS_INFO("  ✓ デッドゾーン制御 (ドリフト防止)");
        ROS_INFO("  ✓ スムーズな加速度制限");
        ROS_INFO("  ✓ 歩行中の動作変更対応");
        ROS_INFO("  ✓ 安全機能 (タイムアウト・緊急停止)");
        ROS_INFO("");
        ROS_INFO("STATUS: 待機中... (L1/LBで有効化)");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hexapod_joystick_controller");
    
    try {
        HexapodJoystickController controller;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in joystick controller: %s", e.what());
        return 1;
    }
    
    return 0;
}
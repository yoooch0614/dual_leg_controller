#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <cmath>

class EnhancedHexapodJoystickController {
public:
    EnhancedHexapodJoystickController() : nh_("~"), last_joy_time_(ros::Time::now()) {
        // パラメータ読み込み
        loadParameters();
        
        // パブリッシャー
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        enable_pub_ = nh_.advertise<std_msgs::Bool>("/hexapod/enable", 1);
        emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("/hexapod/emergency_stop", 1);
        home_pub_ = nh_.advertise<std_msgs::Bool>("/hexapod/home", 1);
        
        // パラメータ調整用パブリッシャー
        step_height_pub_ = nh_.advertise<std_msgs::Float64>("/hexapod/param/step_height", 1);
        step_length_pub_ = nh_.advertise<std_msgs::Float64>("/hexapod/param/step_length", 1);
        cycle_time_pub_ = nh_.advertise<std_msgs::Float64>("/hexapod/param/cycle_time", 1);
        max_speed_pub_ = nh_.advertise<std_msgs::Float64>("/hexapod/param/max_speed", 1);
        
        // サブスクライバー
        joy_sub_ = nh_.subscribe("/joy", 1, &EnhancedHexapodJoystickController::joyCallback, this);
        
        // 制御タイマー（高頻度更新）
        control_timer_ = nh_.createTimer(
            ros::Duration(1.0 / update_rate_), 
            &EnhancedHexapodJoystickController::controlLoop, this);
        
        // 初期化
        current_cmd_.linear.x = 0.0;
        current_cmd_.linear.y = 0.0;
        current_cmd_.angular.z = 0.0;
        
        target_cmd_.linear.x = 0.0;
        target_cmd_.linear.y = 0.0;
        target_cmd_.angular.z = 0.0;
        
        is_enabled_ = false;
        is_stopped_ = true;  // 待機状態フラグを追加
        
        // ボタン状態
        memset(last_buttons_, 0, sizeof(last_buttons_));
        
        // パラメータ調整用変数
        param_adjustment_mode_ = 0;  // 0=速度, 1=ステップ高さ, 2=ステップ長さ, 3=サイクル時間
        current_step_height_ = 17.0;
        current_step_length_ = 60.0;
        current_cycle_time_ = 1.2;
        current_max_speed_ = 80.0;
        
        ROS_INFO("Enhanced Hexapod Joystick Controller initialized");
        printInstructions();
    }

private:
    ros::NodeHandle nh_;
    
    // Publishers
    ros::Publisher cmd_vel_pub_;
    ros::Publisher enable_pub_;
    ros::Publisher emergency_stop_pub_;
    ros::Publisher home_pub_;
    ros::Publisher step_height_pub_;
    ros::Publisher step_length_pub_;
    ros::Publisher cycle_time_pub_;
    ros::Publisher max_speed_pub_;
    
    // Subscribers
    ros::Subscriber joy_sub_;
    
    // Timer
    ros::Timer control_timer_;
    
    // ジョイスティック設定
    double deadzone_radius_;
    double max_linear_speed_;
    double max_angular_speed_;
    double acceleration_limit_;
    double angular_acceleration_limit_;
    double update_rate_;
    
    // ボタン・軸マッピング
    int enable_button_;
    int emergency_stop_button_;
    int home_button_;
    int param_mode_button_;     // パラメータ調整モード切替
    int param_up_button_;       // パラメータ増加
    int param_down_button_;     // パラメータ減少
    
    int linear_x_axis_;
    int linear_y_axis_;
    int angular_z_axis_;
    
    bool invert_linear_x_;
    bool invert_linear_y_;
    bool invert_angular_z_;
    
    // 状態管理
    geometry_msgs::Twist current_cmd_;
    geometry_msgs::Twist target_cmd_;
    sensor_msgs::Joy last_joy_msg_;
    ros::Time last_joy_time_;
    
    bool is_enabled_;
    bool is_stopped_;  // 待機状態フラグ
    bool last_buttons_[16];  // ボタン状態記録用
    
    // パラメータ調整
    int param_adjustment_mode_;
    double current_step_height_;
    double current_step_length_;
    double current_cycle_time_;
    double current_max_speed_;
    
    void loadParameters() {
        // ジョイスティック基本設定
        nh_.param("deadzone_radius", deadzone_radius_, 0.1);
        nh_.param("max_linear_speed", max_linear_speed_, 80.0);
        nh_.param("max_angular_speed", max_angular_speed_, 1.0);
        nh_.param("acceleration_limit", acceleration_limit_, 200.0);
        nh_.param("angular_acceleration_limit", angular_acceleration_limit_, 2.0);
        nh_.param("update_rate", update_rate_, 50.0);
        
        // ボタンマッピング
        nh_.param("enable_button", enable_button_, 4);  // L1/LB
        nh_.param("emergency_stop_button", emergency_stop_button_, 5);  // R1/RB
        nh_.param("home_button", home_button_, 8);  // Start
        nh_.param("param_mode_button", param_mode_button_, 9);  // Select/Back
        nh_.param("param_up_button", param_up_button_, 3);  // Y/Triangle
        nh_.param("param_down_button", param_down_button_, 0);  // A/X
        
        // 軸マッピング
        nh_.param("linear_x_axis", linear_x_axis_, 1);
        nh_.param("linear_y_axis", linear_y_axis_, 0);
        nh_.param("angular_z_axis", angular_z_axis_, 3);
        
        // 軸反転設定
        nh_.param("invert_linear_x", invert_linear_x_, true);
        nh_.param("invert_linear_y", invert_linear_y_, false);
        nh_.param("invert_angular_z", invert_angular_z_, false);
        
        // 歩行パラメータ初期値
        nh_.param("step_height", current_step_height_, 17.0);
        nh_.param("step_length", current_step_length_, 60.0);
        nh_.param("cycle_time", current_cycle_time_, 1.2);
        nh_.param("max_speed", current_max_speed_, 80.0);
        
        ROS_INFO("Enhanced Joystick Controller Parameters loaded:");
        ROS_INFO("  Deadzone radius: %.2f", deadzone_radius_);
        ROS_INFO("  Max linear speed: %.1f mm/s", max_linear_speed_);
        ROS_INFO("  Max angular speed: %.2f rad/s", max_angular_speed_);
        ROS_INFO("  Update rate: %.1f Hz", update_rate_);
    }
    
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
        last_joy_time_ = ros::Time::now();
        last_joy_msg_ = *msg;
        
        // ボタンの状態をチェック（エッジ検出）
        if (msg->buttons.size() >= 16) {
            checkButtonPresses(msg);
        }
        
        // ジョイスティック軸の値を処理（有効化時のみ）
        if (is_enabled_ && 
            msg->axes.size() > std::max({linear_x_axis_, linear_y_axis_, angular_z_axis_})) {
            
            processJoystickAxes(msg);
        } else if (!is_enabled_) {
            // 無効化時は目標値を0に（静止状態を維持）
            target_cmd_.linear.x = 0.0;
            target_cmd_.linear.y = 0.0;
            target_cmd_.angular.z = 0.0;
        }
    }
    
    void checkButtonPresses(const sensor_msgs::Joy::ConstPtr& msg) {
        // Enable/Disable トグル (L1/LB)
        if (msg->buttons[enable_button_] && !last_buttons_[enable_button_]) {
            is_enabled_ = !is_enabled_;
            std_msgs::Bool enable_msg;
            enable_msg.data = is_enabled_;
            enable_pub_.publish(enable_msg);
            
            if (!is_enabled_) {
                // 無効化時は即座に静止状態に
                is_stopped_ = true;
                target_cmd_.linear.x = 0.0;
                target_cmd_.linear.y = 0.0;
                target_cmd_.angular.z = 0.0;
                current_cmd_.linear.x = 0.0;
                current_cmd_.linear.y = 0.0;
                current_cmd_.angular.z = 0.0;
            }
            
            ROS_INFO("Hexapod %s", is_enabled_ ? "ENABLED" : "DISABLED");
        }
        
        // Emergency Stop (R1/RB)
        if (msg->buttons[emergency_stop_button_] && !last_buttons_[emergency_stop_button_]) {
            is_enabled_ = false;
            is_stopped_ = true;
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
        
        // Home Position (Start)
        if (msg->buttons[home_button_] && !last_buttons_[home_button_]) {
            std_msgs::Bool home_msg;
            home_msg.data = true;
            home_pub_.publish(home_msg);
            is_stopped_ = true;
            ROS_INFO("Moving to home position");
        }
        
        // パラメータ調整モード切替 (Select/Back)
        if (msg->buttons[param_mode_button_] && !last_buttons_[param_mode_button_]) {
            param_adjustment_mode_ = (param_adjustment_mode_ + 1) % 4;
            const char* mode_names[] = {"最大速度", "ステップ高さ", "ステップ長さ", "サイクル時間"};
            ROS_INFO("パラメータ調整モード: %s", mode_names[param_adjustment_mode_]);
            printCurrentParameters();
        }
        
        // パラメータ増加 (Y/Triangle)
        if (msg->buttons[param_up_button_] && !last_buttons_[param_up_button_]) {
            adjustParameter(true);
        }
        
        // パラメータ減少 (A/X)
        if (msg->buttons[param_down_button_] && !last_buttons_[param_down_button_]) {
            adjustParameter(false);
        }
        
        // ボタン状態を記録
        for (int i = 0; i < std::min(16, (int)msg->buttons.size()); i++) {
            last_buttons_[i] = msg->buttons[i];
        }
    }
    
    void processJoystickAxes(const sensor_msgs::Joy::ConstPtr& msg) {
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
            
            target_cmd_.linear.x = normalized_x * current_max_speed_ / 1000.0;  // mm/s → m/s
            target_cmd_.linear.y = normalized_y * current_max_speed_ / 1000.0;  // mm/s → m/s
            
            is_stopped_ = false;  // 移動開始
        } else {
            target_cmd_.linear.x = 0.0;
            target_cmd_.linear.y = 0.0;
        }
        
        // 回転のデッドゾーン処理
        if (std::abs(raw_angular) > deadzone_radius_) {
            double angular_scale = (std::abs(raw_angular) - deadzone_radius_) / (1.0 - deadzone_radius_);
            angular_scale = std::min(1.0, angular_scale);
            
            target_cmd_.angular.z = (raw_angular > 0 ? angular_scale : -angular_scale) * max_angular_speed_;
            is_stopped_ = false;  // 回転開始
        } else {
            target_cmd_.angular.z = 0.0;
        }
        
        // 完全に静止状態かチェック
        if (target_cmd_.linear.x == 0.0 && target_cmd_.linear.y == 0.0 && target_cmd_.angular.z == 0.0) {
            is_stopped_ = true;
        }
    }
    
    void adjustParameter(bool increase) {
        double step = increase ? 1.0 : -1.0;
        std_msgs::Float64 param_msg;
        
        switch (param_adjustment_mode_) {
            case 0:  // 最大速度
                current_max_speed_ = std::max(10.0, std::min(150.0, current_max_speed_ + step * 10.0));
                max_linear_speed_ = current_max_speed_;  // 内部変数も更新
                param_msg.data = current_max_speed_;
                max_speed_pub_.publish(param_msg);
                ROS_INFO("最大速度: %.1f mm/s", current_max_speed_);
                break;
                
            case 1:  // ステップ高さ
                current_step_height_ = std::max(5.0, std::min(30.0, current_step_height_ + step * 2.0));
                param_msg.data = current_step_height_;
                step_height_pub_.publish(param_msg);
                ROS_INFO("ステップ高さ: %.1f mm", current_step_height_);
                break;
                
            case 2:  // ステップ長さ
                current_step_length_ = std::max(20.0, std::min(100.0, current_step_length_ + step * 5.0));
                param_msg.data = current_step_length_;
                step_length_pub_.publish(param_msg);
                ROS_INFO("ステップ長さ: %.1f mm", current_step_length_);
                break;
                
            case 3:  // サイクル時間
                current_cycle_time_ = std::max(0.6, std::min(2.5, current_cycle_time_ + step * 0.1));
                param_msg.data = current_cycle_time_;
                cycle_time_pub_.publish(param_msg);
                ROS_INFO("サイクル時間: %.1f s", current_cycle_time_);
                break;
        }
    }
    
    void controlLoop(const ros::TimerEvent& event) {
        // タイムアウトチェック（ジョイスティック接続確認）
        if ((ros::Time::now() - last_joy_time_).toSec() > 1.0) {
            // 1秒以上ジョイスティック入力がない場合は停止
            target_cmd_.linear.x = 0.0;
            target_cmd_.linear.y = 0.0;
            target_cmd_.angular.z = 0.0;
            is_stopped_ = true;
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
        
        // 完全停止状態の判定
        double total_velocity = sqrt(current_cmd_.linear.x * current_cmd_.linear.x + 
                                   current_cmd_.linear.y * current_cmd_.linear.y) + 
                              std::abs(current_cmd_.angular.z);
        
        if (total_velocity < 0.001) {  // ほぼ0の場合
            is_stopped_ = true;
        } else {
            is_stopped_ = false;
        }
        
        // cmd_vel公開（停止時は0のcmd_velを送信して静止状態を確保）
        if (is_stopped_) {
            geometry_msgs::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.linear.y = 0.0;
            stop_cmd.angular.z = 0.0;
            cmd_vel_pub_.publish(stop_cmd);
        } else {
            cmd_vel_pub_.publish(current_cmd_);
        }
    }
    
    void printCurrentParameters() {
        const char* mode_names[] = {"最大速度", "ステップ高さ", "ステップ長さ", "サイクル時間"};
        ROS_INFO("=== 現在のパラメータ ===");
        ROS_INFO("調整モード: %s ← [Select/Backボタンで切替]", mode_names[param_adjustment_mode_]);
        ROS_INFO("  最大速度: %.1f mm/s %s", current_max_speed_, 
                 param_adjustment_mode_ == 0 ? "← [Y/A ボタンで調整]" : "");
        ROS_INFO("  ステップ高さ: %.1f mm %s", current_step_height_,
                 param_adjustment_mode_ == 1 ? "← [Y/A ボタンで調整]" : "");
        ROS_INFO("  ステップ長さ: %.1f mm %s", current_step_length_,
                 param_adjustment_mode_ == 2 ? "← [Y/A ボタンで調整]" : "");
        ROS_INFO("  サイクル時間: %.1f s %s", current_cycle_time_,
                 param_adjustment_mode_ == 3 ? "← [Y/A ボタンで調整]" : "");
    }
    
    void printInstructions() {
        ROS_INFO("=== ENHANCED HEXAPOD JOYSTICK CONTROLLER ===");
        ROS_INFO("6脚ロボット ジョイスティック制御システム（強化版）");
        ROS_INFO("");
        ROS_INFO("FEATURES:");
        ROS_INFO("  ✓ 待機時完全静止（足の無駄な動きを停止）");
        ROS_INFO("  ✓ リアルタイムパラメータ調整");
        ROS_INFO("  ✓ スムーズな加速度制限");
        ROS_INFO("  ✓ 改良されたデッドゾーン処理");
        ROS_INFO("");
        ROS_INFO("CONTROLS:");
        ROS_INFO("  L1/LB: システム有効化/無効化");
        ROS_INFO("  左スティック: 移動制御（前後・左右）");
        ROS_INFO("  右スティック: 回転制御");
        ROS_INFO("  R1/RB: 緊急停止");
        ROS_INFO("  Start: ホームポジション");
        ROS_INFO("");
        ROS_INFO("PARAMETER ADJUSTMENT:");
        ROS_INFO("  Select/Back: パラメータ調整モード切替");
        ROS_INFO("  Y/Triangle: パラメータ増加");
        ROS_INFO("  A/X: パラメータ減少");
        ROS_INFO("");
        ROS_INFO("STATUS: システム初期化完了 - 待機中");
        printCurrentParameters();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "enhanced_hexapod_joystick_controller");
    
    try {
        EnhancedHexapodJoystickController hexapod;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in enhanced hexapod joystick controller: %s", e.what());
        return 1;
    }
    
    return 0;
}
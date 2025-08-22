#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <dual_leg_controller/LegPosition.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
#include <map>
#include <vector>
#include <string>

class EnhancedSmoothHexapodController {
private:
    struct LegConfig {
        std::string leg_id;
        double attach_angle;      // 取り付け角度（度）
        int tripod_group;        // トライポッドグループ (0 or 1)
        double home_x, home_y, home_z;
        double current_x, current_y, current_z;
        double phase_offset;     // 位相オフセット
        bool was_walking;
        bool is_swing_phase;
        double step_progress;    // 現在のステップ進行度 (0.0-1.0)
    };
    
    struct WalkParams {
        double step_height;
        double step_length;
        double cycle_time;
        double stance_time_ratio;  // スタンス期の割合
    };
    
    struct SmoothTransition {
        double current_direction;  // 現在の方向（度）
        double target_direction;   // 目標方向（度）
        double current_speed;      // 現在の速度（mm/s）
        double target_speed;       // 目標速度（mm/s）
        double current_angular;    // 現在の角速度（rad/s）
        double target_angular;     // 目標角速度（rad/s）
        double direction_change_rate;  // 方向変更レート（度/秒）
        double speed_change_rate;      // 速度変更レート（mm/s²）
        double angular_change_rate;    // 角速度変更レート（rad/s²）
    };
    
    ros::NodeHandle nh_;
    std::map<std::string, ros::Publisher> leg_pos_pubs_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber enable_sub_;
    ros::Subscriber emergency_stop_sub_;
    ros::Subscriber home_sub_;
    ros::Timer control_timer_;
    
    std::map<std::string, LegConfig> legs_;
    std::vector<std::string> leg_names_;
    WalkParams walk_params_;
    SmoothTransition smooth_transition_;
    
    bool is_walking_;
    bool is_enabled_;
    bool emergency_stop_;
    double current_time_;
    double last_cmd_time_;
    
    // 歩行中断と再開のための変数
    bool allow_gait_interruption_;
    double min_step_completion_;
    std::map<std::string, double> leg_step_start_time_;

public:
    EnhancedSmoothHexapodController() : nh_("~"), is_walking_(false), is_enabled_(false), 
                                       emergency_stop_(false), current_time_(0.0), 
                                       last_cmd_time_(0.0) {
        
        // パラメータ読み込み
        loadParameters();
        
        // 6脚設定
        setupHexapodConfiguration();
        
        // パブリッシャー設定
        for (const auto& leg_pair : legs_) {
            const std::string& leg_id = leg_pair.first;
            std::string topic = "/asterisk/leg/" + leg_id + "/command/foot_position";
            leg_pos_pubs_[leg_id] = nh_.advertise<dual_leg_controller::LegPosition>(topic, 1);
        }
        
        // サブスクライバー設定
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, 
                                   &EnhancedSmoothHexapodController::cmdVelCallback, this);
        enable_sub_ = nh_.subscribe("/hexapod/enable", 1,
                                  &EnhancedSmoothHexapodController::enableCallback, this);
        emergency_stop_sub_ = nh_.subscribe("/hexapod/emergency_stop", 1,
                                          &EnhancedSmoothHexapodController::emergencyStopCallback, this);
        home_sub_ = nh_.subscribe("/hexapod/home", 1,
                                &EnhancedSmoothHexapodController::homeCallback, this);
        
        // 制御タイマー（50Hz）
        control_timer_ = nh_.createTimer(ros::Duration(0.02), 
                                       &EnhancedSmoothHexapodController::controlCallback, this);
        
        ROS_INFO("Enhanced Smooth Hexapod Controller initialized");
        printInstructions();
        moveToHome();
    }

private:
    void loadParameters() {
        // 歩行パラメータ
        nh_.param("step_height", walk_params_.step_height, 17.0);
        nh_.param("step_length", walk_params_.step_length, 60.0);
        nh_.param("cycle_time", walk_params_.cycle_time, 1.2);
        nh_.param("stance_time_ratio", walk_params_.stance_time_ratio, 0.6);
        
        // スムーズ遷移パラメータ
        nh_.param("direction_change_rate", smooth_transition_.direction_change_rate, 120.0);
        nh_.param("speed_change_rate", smooth_transition_.speed_change_rate, 80.0);
        nh_.param("angular_change_rate", smooth_transition_.angular_change_rate, 1.5);
        
        // 歩行中断・再開パラメータ
        nh_.param("allow_gait_interruption", allow_gait_interruption_, true);
        nh_.param("min_step_completion", min_step_completion_, 0.3);
        
        // 初期値設定
        smooth_transition_.current_direction = 0.0;
        smooth_transition_.target_direction = 0.0;
        smooth_transition_.current_speed = 0.0;
        smooth_transition_.target_speed = 0.0;
        smooth_transition_.current_angular = 0.0;
        smooth_transition_.target_angular = 0.0;
        
        ROS_INFO("Enhanced Controller Parameters loaded:");
        ROS_INFO("  Step: height=%.1fmm, length=%.1fmm, cycle=%.1fs", 
                 walk_params_.step_height, walk_params_.step_length, walk_params_.cycle_time);
        ROS_INFO("  Smooth transition rates: dir=%.1f°/s, speed=%.1fmm/s², ang=%.1frad/s²",
                 smooth_transition_.direction_change_rate, smooth_transition_.speed_change_rate,
                 smooth_transition_.angular_change_rate);
    }
    
    void setupHexapodConfiguration() {
        leg_names_ = {"RF", "LF", "LM", "LB", "RB", "RM"};
        
        // 各脚の設定（実際の配置に基づく）
        double attach_angles[6] = {0, 300, 240, 180, 120, 60};  // 度
        int tripod_groups[6] = {0, 1, 0, 1, 0, 1};
        
        for (int i = 0; i < 6; i++) {
            LegConfig config;
            config.leg_id = leg_names_[i];
            config.attach_angle = attach_angles[i];
            config.tripod_group = tripod_groups[i];
            
            // ホームポジション設定
            config.home_x = 140.0;
            config.home_y = 0.0;
            config.home_z = -90.0;
            
            // 現在位置をホームに初期化
            config.current_x = config.home_x;
            config.current_y = config.home_y;
            config.current_z = config.home_z;
            
            // 位相オフセット（トライポッドグループで0.5周期ずらす）
            config.phase_offset = config.tripod_group * 0.5;
            config.was_walking = false;
            config.is_swing_phase = false;
            config.step_progress = 0.0;
            
            legs_[config.leg_id] = config;
        }
        
        ROS_INFO("Hexapod configuration completed - 6 legs in 2 tripod groups");
    }
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        if (emergency_stop_) return;
        
        last_cmd_time_ = ros::Time::now().toSec();
        
        // cmd_velから目標値を計算
        double linear_speed = sqrt(msg->linear.x * msg->linear.x + msg->linear.y * msg->linear.y) * 1000.0; // m/s → mm/s
        double direction = atan2(msg->linear.y, msg->linear.x) * 180.0 / M_PI; // ラジアン → 度
        
        // 目標値を設定
        smooth_transition_.target_speed = linear_speed;
        smooth_transition_.target_angular = msg->angular.z;
        
        if (linear_speed > 1.0) { // 移動コマンドがある場合のみ方向を更新
            smooth_transition_.target_direction = direction;
        }
        
        // 歩行開始判定
        if (!is_walking_ && is_enabled_ && 
            (linear_speed > 1.0 || std::abs(msg->angular.z) > 0.01)) {
            startWalking();
        }
        
        ROS_DEBUG("cmd_vel: speed=%.1fmm/s, dir=%.1f°, angular=%.3frad/s", 
                  linear_speed, direction, msg->angular.z);
    }
    
    void enableCallback(const std_msgs::Bool::ConstPtr& msg) {
        is_enabled_ = msg->data;
        if (!is_enabled_) {
            stopWalking();
        }
        ROS_INFO("Hexapod %s", is_enabled_ ? "ENABLED" : "DISABLED");
    }
    
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data) {
            emergency_stop_ = true;
            is_enabled_ = false;
            is_walking_ = false;
            
            // 即座に現在位置で停止
            for (auto& leg_pair : legs_) {
                LegConfig& config = leg_pair.second;
                dual_leg_controller::LegPosition cmd;
                cmd.x = config.current_x;
                cmd.y = config.current_y;
                cmd.z = config.current_z;
                leg_pos_pubs_[config.leg_id].publish(cmd);
            }
            
            ROS_WARN("EMERGENCY STOP ACTIVATED - System halted");
        } else {
            emergency_stop_ = false;
            ROS_INFO("Emergency stop released");
        }
    }
    
    void homeCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data && !emergency_stop_) {
            moveToHome();
        }
    }
    
    void startWalking() {
        is_walking_ = true;
        current_time_ = 0.0;
        
        // 各脚の歩行開始時刻を記録
        for (auto& leg_pair : legs_) {
            LegConfig& config = leg_pair.second;
            leg_step_start_time_[config.leg_id] = current_time_;
            config.was_walking = true;
        }
        
        ROS_INFO("Walking started - Smooth transition enabled");
    }
    
    void stopWalking() {
        if (!is_walking_) return;
        
        // 滑らかに停止するため、目標速度を0に設定
        smooth_transition_.target_speed = 0.0;
        smooth_transition_.target_angular = 0.0;
        
        ROS_INFO("Walking stop requested - Decelerating smoothly");
    }
    
    void controlCallback(const ros::TimerEvent& event) {
        if (emergency_stop_) return;
        
        current_time_ += 0.02; // 50Hz
        
        // タイムアウトチェック（1秒間cmd_velがない場合は停止）
        if (is_walking_ && (ros::Time::now().toSec() - last_cmd_time_) > 1.0) {
            stopWalking();
        }
        
        // スムーズな遷移処理
        updateSmoothTransition();
        
        if (is_walking_ && is_enabled_) {
            updateWalkingGait();
        } else if (smooth_transition_.current_speed < 1.0) {
            // 完全に停止した場合
            if (is_walking_) {
                is_walking_ = false;
                ROS_INFO("Walking stopped - All legs at rest");
            }
        }
    }
    
    void updateSmoothTransition() {
        double dt = 0.02; // 50Hz
        
        // 速度の滑らかな変更
        double speed_diff = smooth_transition_.target_speed - smooth_transition_.current_speed;
        double max_speed_change = smooth_transition_.speed_change_rate * dt;
        
        if (std::abs(speed_diff) > max_speed_change) {
            smooth_transition_.current_speed += (speed_diff > 0 ? max_speed_change : -max_speed_change);
        } else {
            smooth_transition_.current_speed = smooth_transition_.target_speed;
        }
        
        // 方向の滑らかな変更
        double dir_diff = smooth_transition_.target_direction - smooth_transition_.current_direction;
        
        // 角度差の正規化（-180°〜+180°）
        while (dir_diff > 180.0) dir_diff -= 360.0;
        while (dir_diff < -180.0) dir_diff += 360.0;
        
        double max_dir_change = smooth_transition_.direction_change_rate * dt;
        
        if (std::abs(dir_diff) > max_dir_change) {
            smooth_transition_.current_direction += (dir_diff > 0 ? max_dir_change : -max_dir_change);
        } else {
            smooth_transition_.current_direction = smooth_transition_.target_direction;
        }
        
        // 角速度の滑らかな変更
        double angular_diff = smooth_transition_.target_angular - smooth_transition_.current_angular;
        double max_angular_change = smooth_transition_.angular_change_rate * dt;
        
        if (std::abs(angular_diff) > max_angular_change) {
            smooth_transition_.current_angular += (angular_diff > 0 ? max_angular_change : -max_angular_change);
        } else {
            smooth_transition_.current_angular = smooth_transition_.target_angular;
        }
    }
    
    void updateWalkingGait() {
        for (auto& leg_pair : legs_) {
            LegConfig& config = leg_pair.second;
            
            // 各脚の位相計算（トライポッドグループ考慮）
            double phase = fmod(current_time_ / walk_params_.cycle_time + config.phase_offset, 1.0);
            
            // 歩行中断と再開の処理
            bool can_change_direction = true;
            if (allow_gait_interruption_) {
                // 最小ステップ完了率をチェック
                double step_completion = fmod(phase, 1.0);
                if (step_completion < min_step_completion_ && config.was_walking) {
                    can_change_direction = false;
                }
            }
            
            // 足先位置計算
            double x, y, z;
            calculateLegPosition(config, phase, can_change_direction, x, y, z);
            
            // 位置コマンド送信
            dual_leg_controller::LegPosition cmd;
            cmd.x = x;
            cmd.y = y;
            cmd.z = z;
            
            // 現在位置を更新
            config.current_x = x;
            config.current_y = y;
            config.current_z = z;
            config.step_progress = phase;
            
            leg_pos_pubs_[config.leg_id].publish(cmd);
        }
        
        // デバッグ情報出力（1秒間隔）
        static double last_debug_time = 0.0;
        if (current_time_ - last_debug_time > 1.0) {
            ROS_INFO("Walking: dir=%.1f°→%.1f°, speed=%.1f→%.1fmm/s, angular=%.3f→%.3frad/s",
                     smooth_transition_.current_direction, smooth_transition_.target_direction,
                     smooth_transition_.current_speed, smooth_transition_.target_speed,
                     smooth_transition_.current_angular, smooth_transition_.target_angular);
            last_debug_time = current_time_;
        }
    }
    
    void calculateLegPosition(LegConfig& config, double phase, bool can_change_direction,
                             double& x, double& y, double& z) {
        
        // スイング期とスタンス期の判定
        bool is_swing = phase < (1.0 - walk_params_.stance_time_ratio);
        config.is_swing_phase = is_swing;
        
        if (is_swing) {
            // スイング期：足を持ち上げて前進
            double swing_progress = phase / (1.0 - walk_params_.stance_time_ratio);
            
            // 軌道計算
            double step_distance = smooth_transition_.current_speed * walk_params_.cycle_time;
            double step_x = step_distance * cos(smooth_transition_.current_direction * M_PI / 180.0);
            double step_y = step_distance * sin(smooth_transition_.current_direction * M_PI / 180.0);
            
            // 回転成分を追加
            double angular_step = smooth_transition_.current_angular * walk_params_.cycle_time;
            double leg_radius = sqrt(config.home_x * config.home_x + config.home_y * config.home_y);
            double angular_x = -leg_radius * sin(config.attach_angle * M_PI / 180.0) * angular_step;
            double angular_y = leg_radius * cos(config.attach_angle * M_PI / 180.0) * angular_step;
            
            // X, Y位置（放物線軌道）
            x = config.home_x + (step_x + angular_x) * (swing_progress - 0.5);
            y = config.home_y + (step_y + angular_y) * (swing_progress - 0.5);
            
            // Z位置（放物線軌道で持ち上げ）
            double lift_height = walk_params_.step_height * 4.0 * swing_progress * (1.0 - swing_progress);
            z = config.home_z + lift_height;
            
        } else {
            // スタンス期：地面に接地して推進
            double stance_progress = (phase - (1.0 - walk_params_.stance_time_ratio)) / walk_params_.stance_time_ratio;
            
            // 地面を蹴って後方に移動
            double step_distance = smooth_transition_.current_speed * walk_params_.cycle_time;
            double step_x = step_distance * cos(smooth_transition_.current_direction * M_PI / 180.0);
            double step_y = step_distance * sin(smooth_transition_.current_direction * M_PI / 180.0);
            
            // 回転成分
            double angular_step = smooth_transition_.current_angular * walk_params_.cycle_time;
            double leg_radius = sqrt(config.home_x * config.home_x + config.home_y * config.home_y);
            double angular_x = -leg_radius * sin(config.attach_angle * M_PI / 180.0) * angular_step;
            double angular_y = leg_radius * cos(config.attach_angle * M_PI / 180.0) * angular_step;
            
            x = config.home_x + (step_x + angular_x) * (0.5 - stance_progress);
            y = config.home_y + (step_y + angular_y) * (0.5 - stance_progress);
            z = config.home_z; // 地面に接地
        }
    }
    
    void moveToHome() {
        is_walking_ = false;
        is_enabled_ = false;
        smooth_transition_.target_speed = 0.0;
        smooth_transition_.current_speed = 0.0;
        smooth_transition_.target_angular = 0.0;
        smooth_transition_.current_angular = 0.0;
        
        for (auto& leg_pair : legs_) {
            LegConfig& config = leg_pair.second;
            
            dual_leg_controller::LegPosition cmd;
            cmd.x = config.home_x;
            cmd.y = config.home_y;
            cmd.z = config.home_z;
            
            config.current_x = cmd.x;
            config.current_y = cmd.y;
            config.current_z = cmd.z;
            config.was_walking = false;
            
            leg_pos_pubs_[config.leg_id].publish(cmd);
        }
        
        ROS_INFO("All legs moved to home position");
    }
    
    void printInstructions() {
        ROS_INFO("=== ENHANCED SMOOTH HEXAPOD CONTROLLER ===");
        ROS_INFO("6脚ロボット スムーズ歩行制御システム");
        ROS_INFO("");
        ROS_INFO("FEATURES:");
        ROS_INFO("  ✓ ジョイスティック制御対応");
        ROS_INFO("  ✓ 歩行中のスムーズな方向変更");
        ROS_INFO("  ✓ 速度・方向・回転の滑らかな遷移");
        ROS_INFO("  ✓ 歩行中断・再開機能");
        ROS_INFO("  ✓ 緊急停止・安全機能");
        ROS_INFO("");
        ROS_INFO("TOPICS:");
        ROS_INFO("  /cmd_vel - 移動コマンド入力");
        ROS_INFO("  /hexapod/enable - 有効化・無効化");
        ROS_INFO("  /hexapod/emergency_stop - 緊急停止");
        ROS_INFO("  /hexapod/home - ホームポジション");
        ROS_INFO("");
        ROS_INFO("STATUS: システム初期化完了");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "enhanced_smooth_hexapod_controller");
    
    try {
        EnhancedSmoothHexapodController hexapod;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in enhanced smooth hexapod controller: %s", e.what());
        return 1;
    }
    
    return 0;
}
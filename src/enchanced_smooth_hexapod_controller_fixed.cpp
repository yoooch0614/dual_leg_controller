#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include "dual_leg_controller/LegPosition.h"
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

struct WalkParameters {
    double step_height;
    double step_length;
    double cycle_time;
    double stance_time_ratio;
};

struct SmoothTransition {
    double current_direction;
    double target_direction;
    double current_speed;
    double target_speed;
    double current_angular;
    double target_angular;
    double direction_change_rate;
    double speed_change_rate;
    double angular_change_rate;
};

struct RobotCommand {
    double velocity_x;
    double velocity_y;
    double angular_z;
};

struct LegConfig {
    std::string leg_id;
    double attach_angle;
    int tripod_group;
    double phase_offset;
    double home_x, home_y, home_z;
    double current_x, current_y, current_z;
    double step_progress;
    bool was_walking;
};

class EnhancedSmoothHexapodController {
private:
    ros::NodeHandle nh_;
    
    // Publishers
    std::map<std::string, ros::Publisher> leg_pos_pubs_;
    
    // Subscribers
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber enable_sub_;
    ros::Subscriber emergency_stop_sub_;
    ros::Subscriber home_sub_;
    ros::Subscriber param_height_sub_;
    ros::Subscriber param_length_sub_;
    ros::Subscriber param_cycle_sub_;
    ros::Subscriber param_speed_sub_;
    
    // Timer
    ros::Timer control_timer_;
    
    // 6脚設定
    std::vector<std::string> leg_names_;
    std::map<std::string, LegConfig> legs_;
    
    // 歩行パラメータ
    WalkParameters walk_params_;
    SmoothTransition smooth_transition_;
    RobotCommand robot_cmd_;
    
    // 状態管理
    bool is_walking_;
    bool is_enabled_;
    bool emergency_stop_;
    bool is_completely_stopped_;  // 完全停止状態フラグを追加
    double current_time_;
    double last_cmd_time_;
    
    // 歩行中断と再開のための変数
    bool allow_gait_interruption_;
    double min_step_completion_;
    std::map<std::string, double> leg_step_start_time_;

public:
    EnhancedSmoothHexapodController() : nh_("~"), is_walking_(false), is_enabled_(false), 
                                       emergency_stop_(false), is_completely_stopped_(true),
                                       current_time_(0.0), last_cmd_time_(0.0) {
        
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
                                
        // パラメータ調整用サブスクライバー
        param_height_sub_ = nh_.subscribe("/hexapod/param/step_height", 1,
                                        &EnhancedSmoothHexapodController::paramHeightCallback, this);
        param_length_sub_ = nh_.subscribe("/hexapod/param/step_length", 1,
                                        &EnhancedSmoothHexapodController::paramLengthCallback, this);
        param_cycle_sub_ = nh_.subscribe("/hexapod/param/cycle_time", 1,
                                       &EnhancedSmoothHexapodController::paramCycleCallback, this);
        param_speed_sub_ = nh_.subscribe("/hexapod/param/max_speed", 1,
                                       &EnhancedSmoothHexapodController::paramSpeedCallback, this);
        
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
        nh_.param("stance_time_ratio", walk_params_.stance_time_ratio, 0.5);
        
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
        
        // 各脚の設定
        double attach_angles[6] = {0, 300, 240, 180, 120, 60};  // 度
        int tripod_groups[6] = {0, 1, 0, 1, 0, 1};
        
        for (int i = 0; i < 6; i++) {
            LegConfig config;
            config.leg_id = leg_names_[i];
            config.attach_angle = attach_angles[i];
            config.tripod_group = tripod_groups[i];
            
            // トライポッドグループによる位相オフセット
            config.phase_offset = (config.tripod_group == 0) ? 0.0 : 0.5;
            
            // ホームポジション設定
            config.home_x = 140.0;  // mm
            config.home_y = 0.0;
            config.home_z = -85.0;
            
            config.current_x = config.home_x;
            config.current_y = config.home_y;
            config.current_z = config.home_z;
            config.step_progress = 0.0;
            config.was_walking = false;
            
            legs_[config.leg_id] = config;
        }
        
        ROS_INFO("Hexapod configuration setup complete - 6 legs configured");
    }
    
    // パラメータ調整用コールバック
    void paramHeightCallback(const std_msgs::Float64::ConstPtr& msg) {
        walk_params_.step_height = std::max(5.0, std::min(30.0, msg->data));
        ROS_INFO("Step height updated: %.1f mm", walk_params_.step_height);
    }
    
    void paramLengthCallback(const std_msgs::Float64::ConstPtr& msg) {
        walk_params_.step_length = std::max(20.0, std::min(100.0, msg->data));
        ROS_INFO("Step length updated: %.1f mm", walk_params_.step_length);
    }
    
    void paramCycleCallback(const std_msgs::Float64::ConstPtr& msg) {
        walk_params_.cycle_time = std::max(0.6, std::min(2.5, msg->data));
        ROS_INFO("Cycle time updated: %.1f s", walk_params_.cycle_time);
    }
    
    void paramSpeedCallback(const std_msgs::Float64::ConstPtr& msg) {
        // 最大速度の動的調整はsmooth_transitionで管理
        ROS_INFO("Max speed parameter received: %.1f mm/s", msg->data);
    }
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        if (emergency_stop_ || !is_enabled_) return;
        
        last_cmd_time_ = ros::Time::now().toSec();
        
        // 入力値から目標値を計算
        double linear_speed = sqrt(msg->linear.x * msg->linear.x + msg->linear.y * msg->linear.y) * 1000.0;  // m/s → mm/s
        double direction = atan2(msg->linear.y, msg->linear.x) * 180.0 / M_PI;  // ラジアン → 度
        
        // 目標値設定
        smooth_transition_.target_speed = linear_speed;
        smooth_transition_.target_direction = direction;
        smooth_transition_.target_angular = msg->angular.z;
        
        // 歩行開始判定
        if (!is_walking_ && (linear_speed > 1.0 || std::abs(msg->angular.z) > 0.01)) {
            startWalking();
        }
        
        // 完全停止判定の更新
        if (linear_speed < 0.1 && std::abs(msg->angular.z) < 0.01) {
            if (is_walking_ && smooth_transition_.current_speed < 1.0) {
                stopWalkingCompletely();
            }
        } else {
            is_completely_stopped_ = false;
        }
    }
    
    void enableCallback(const std_msgs::Bool::ConstPtr& msg) {
        is_enabled_ = msg->data;
        
        if (!is_enabled_) {
            // 無効化時は即座に完全停止
            stopWalkingCompletely();
        }
        
        ROS_INFO("Hexapod %s", is_enabled_ ? "ENABLED" : "DISABLED");
    }
    
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data) {
            emergency_stop_ = true;
            is_enabled_ = false;
            is_walking_ = false;
            is_completely_stopped_ = true;
            
            // 即座に現在位置で静止
            for (auto& leg_pair : legs_) {
                LegConfig& config = leg_pair.second;
                dual_leg_controller::LegPosition cmd;
                cmd.x = config.current_x;
                cmd.y = config.current_y;
                cmd.z = config.current_z;
                leg_pos_pubs_[config.leg_id].publish(cmd);
            }
            
            ROS_WARN("EMERGENCY STOP ACTIVATED - All legs stopped immediately");
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
        is_completely_stopped_ = false;
        current_time_ = 0.0;
        
        // 各脚の歩行開始時刻を記録
        for (auto& leg_pair : legs_) {
            LegConfig& config = leg_pair.second;
            leg_step_start_time_[config.leg_id] = current_time_;
            config.was_walking = true;
        }
        
        ROS_INFO("Walking started - Smooth transition enabled");
    }
    
    void stopWalkingCompletely() {
        if (!is_walking_) return;
        
        is_walking_ = false;
        is_completely_stopped_ = true;
        
        // 即座に全ての目標値を0に設定
        smooth_transition_.target_speed = 0.0;
        smooth_transition_.target_angular = 0.0;
        smooth_transition_.current_speed = 0.0;
        smooth_transition_.current_angular = 0.0;
        
        // 全ての脚をホームポジションに移動（静止状態）
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
        
        ROS_INFO("Walking stopped completely - All legs at home position and stationary");
    }
    
    void controlCallback(const ros::TimerEvent& event) {
        if (emergency_stop_) return;
        
        current_time_ += 0.02; // 50Hz
        
        // タイムアウトチェック（1秒間cmd_velがない場合は完全停止）
        if (is_walking_ && (ros::Time::now().toSec() - last_cmd_time_) > 1.0) {
            stopWalkingCompletely();
        }
        
        // 完全停止状態の場合は足の動きを更新しない
        if (is_completely_stopped_) {
            return;  // 何もしない（足は静止状態を維持）
        }
        
        // スムーズな遷移処理
        updateSmoothTransition();
        
        // 歩行制御（有効かつ歩行中のみ）
        if (is_walking_ && is_enabled_) {
            updateWalkingGait();
        }
        
        // 完全停止判定
        if (smooth_transition_.current_speed < 0.5 && std::abs(smooth_transition_.current_angular) < 0.01) {
            if (is_walking_) {
                stopWalkingCompletely();
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
        
        // robot_cmd_を更新
        robot_cmd_.velocity_x = smooth_transition_.current_speed * cos(smooth_transition_.current_direction * M_PI / 180.0);
        robot_cmd_.velocity_y = smooth_transition_.current_speed * sin(smooth_transition_.current_direction * M_PI / 180.0);
        robot_cmd_.angular_z = smooth_transition_.current_angular;
    }
    
    void updateWalkingGait() {
        for (auto& leg_pair : legs_) {
            LegConfig& config = leg_pair.second;
            
            // 各脚の位相計算（トライポッドグループ考慮）
            double phase = fmod(current_time_ / walk_params_.cycle_time + config.phase_offset, 1.0);
            
            // 足先位置計算
            double x, y, z;
            calculateLegPositionTripod(config, phase, x, y, z);
            
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
        
        // デバッグ情報出力（2秒間隔）
        static double last_debug_time = 0.0;
        if (current_time_ - last_debug_time > 2.0) {
            ROS_INFO("Walking: speed=%.1fmm/s, dir=%.1f°, angular=%.3frad/s",
                     smooth_transition_.current_speed,
                     smooth_transition_.current_direction,
                     smooth_transition_.current_angular);
            last_debug_time = current_time_;
        }
    }
    
    void calculateLegPositionTripod(const LegConfig& config, double phase, 
                                   double& x, double& y, double& z) {
        // ロボット座標系の移動量を脚座標系に変換
        double angle_rad = config.attach_angle * M_PI / 180.0;
        
        // 座標変換：ロボット座標系 → 脚座標系
        double leg_velocity_x =  robot_cmd_.velocity_x * cos(angle_rad) + 
                                robot_cmd_.velocity_y * sin(angle_rad);
        double leg_velocity_y = -robot_cmd_.velocity_x * sin(angle_rad) + 
                                robot_cmd_.velocity_y * cos(angle_rad);
        
        // 回転成分の追加（ロボット中心からの距離を95mmと仮定）
        double body_radius = 95.0;  // mm
        double rotation_velocity_x = -robot_cmd_.angular_z * body_radius * sin(angle_rad);
        double rotation_velocity_y =  robot_cmd_.angular_z * body_radius * cos(angle_rad);
        
        // 合成速度
        leg_velocity_x += rotation_velocity_x;
        leg_velocity_y += rotation_velocity_y;
        
        // ステップ幅計算
        double velocity_magnitude = sqrt(leg_velocity_x * leg_velocity_x + leg_velocity_y * leg_velocity_y);
        double leg_step_x, leg_step_y;
        
        if (velocity_magnitude > 0.1) {
            // 方向は保持、大きさを step_length に設定
            leg_step_x = (leg_velocity_x / velocity_magnitude) * walk_params_.step_length;
            leg_step_y = (leg_velocity_y / velocity_magnitude) * walk_params_.step_length;
        } else {
            // 停止時
            leg_step_x = 0.0;
            leg_step_y = 0.0;
        }
        
        // スイング/スタンス期の計算
        bool is_swing_phase = (phase < 0.5);
        double step_progress, x_offset, y_offset, z_offset;
        
        if (is_swing_phase) {
            // スイング期：前方移動（空中）
            step_progress = phase * 2.0;  // 0→1
            x_offset = -leg_step_x * 0.5 + leg_step_x * step_progress;
            y_offset = -leg_step_y * 0.5 + leg_step_y * step_progress;
            
            // 放物線軌道でスイング
            double swing_height = 4.0 * walk_params_.step_height * step_progress * (1.0 - step_progress);
            z_offset = swing_height;
        } else {
            // スタンス期：後方移動（地面接触でロボットを押す）
            step_progress = (phase - 0.5) * 2.0;  // 0→1
            x_offset = leg_step_x * 0.5 - leg_step_x * step_progress;
            y_offset = leg_step_y * 0.5 - leg_step_y * step_progress;
            z_offset = 0.0;
        }
        
        // 最終的な足先位置（脚座標系）
        x = config.home_x + x_offset;
        y = config.home_y + y_offset;
        z = config.home_z + z_offset;
    }
    
    void moveToHome() {
        is_walking_ = false;
        is_enabled_ = false;
        is_completely_stopped_ = true;
        
        // 遷移パラメータをリセット
        smooth_transition_.target_speed = 0.0;
        smooth_transition_.current_speed = 0.0;
        smooth_transition_.target_angular = 0.0;
        smooth_transition_.current_angular = 0.0;
        
        // 全ての脚をホームポジションに移動
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
        
        ROS_INFO("All legs moved to home position and stopped completely");
    }
    
    void printInstructions() {
        ROS_INFO("=== ENHANCED SMOOTH HEXAPOD CONTROLLER (FIXED) ===");
        ROS_INFO("6脚ロボット スムーズ歩行制御システム（待機時静止対応版）");
        ROS_INFO("");
        ROS_INFO("FEATURES:");
        ROS_INFO("  ✓ 待機時完全静止（足の無駄な動きを完全に停止）");
        ROS_INFO("  ✓ ジョイスティックによるリアルタイムパラメータ調整");
        ROS_INFO("  ✓ 速度・方向・回転の滑らかな遷移");
        ROS_INFO("  ✓ 歩行中断・再開機能");
        ROS_INFO("  ✓ 緊急停止・安全機能");
        ROS_INFO("");
        ROS_INFO("JOYSTICK CONTROLS:");
        ROS_INFO("  L1/LB: システム有効化/無効化");
        ROS_INFO("  左スティック: 移動制御（前後・左右）");
        ROS_INFO("  右スティック: 回転制御");
        ROS_INFO("  R1/RB: 緊急停止");
        ROS_INFO("  Start: ホームポジション");
        ROS_INFO("");
        ROS_INFO("PARAMETER ADJUSTMENT:");
        ROS_INFO("  Select/Back: パラメータ調整モード切替");
        ROS_INFO("  Y/Triangle: 選択中パラメータ増加");
        ROS_INFO("  A/X: 選択中パラメータ減少");
        ROS_INFO("");
        ROS_INFO("TOPICS:");
        ROS_INFO("  /cmd_vel - 移動コマンド入力");
        ROS_INFO("  /hexapod/enable - 有効化・無効化");
        ROS_INFO("  /hexapod/emergency_stop - 緊急停止");
        ROS_INFO("  /hexapod/home - ホームポジション");
        ROS_INFO("  /hexapod/param/* - パラメータ調整");
        ROS_INFO("");
        ROS_INFO("STATUS: システム初期化完了 - 完全静止状態");
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
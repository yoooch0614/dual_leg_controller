#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <dual_leg_controller/LegPosition.h>
#include <cmath>
#include <map>
#include <vector>
#include <string>

class EnhancedSmoothHexapodController {
private:
    struct LegConfig {
        std::string leg_id;
        double attach_angle;      // 取り付け角度（度）
        int tripod_group;         // トライポッドグループ (0 or 1)
        double home_x, home_y, home_z;
        double current_x, current_y, current_z;
        double phase_offset;      // 位相オフセット
        bool was_walking;
        bool is_swing_phase;
        double step_progress;     // 現在のステップ進行度 (0.0-1.0)
        double stop_x, stop_y, stop_z;  // 停止時の位置を記録
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
    
    struct RobotCommand {
        double velocity_x = 0.0;  // mm/s
        double velocity_y = 0.0;  // mm/s
        double angular_z = 0.0;   // rad/s
    } robot_cmd_;
    
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
    bool is_completely_stopped_;  // 完全停止状態フラグ
    bool is_stopping_;            // 停止中フラグ
    double current_time_;
    double last_cmd_time_;
    double stop_start_time_;      // 停止開始時刻
    double walk_start_time_;      // 歩行開始時刻
    
    // 歩行中断と再開のための変数
    bool allow_gait_interruption_;
    double min_step_completion_;
    std::map<std::string, double> leg_step_start_time_;
    
    // 静止判定用の閾値
    double zero_velocity_threshold_;
    double idle_timeout_;
    double deceleration_time_;    // 減速にかける時間
    double startup_blend_time_;   // 起動時のブレンド時間

public:
    EnhancedSmoothHexapodController() : nh_("~"), 
                                       is_walking_(false), 
                                       is_enabled_(false), 
                                       emergency_stop_(false), 
                                       is_completely_stopped_(true),
                                       is_stopping_(false),
                                       current_time_(0.0), 
                                       last_cmd_time_(0.0),
                                       stop_start_time_(0.0),
                                       walk_start_time_(0.0) {
        
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
        nh_.param("stance_time_ratio", walk_params_.stance_time_ratio, 0.5);
        
        // スムーズ遷移パラメータ
        nh_.param("direction_change_rate", smooth_transition_.direction_change_rate, 120.0);  // 度/秒
        nh_.param("speed_change_rate", smooth_transition_.speed_change_rate, 120.0);          // mm/s² (高速化)
        nh_.param("angular_change_rate", smooth_transition_.angular_change_rate, 2.0);        // rad/s²
        
        // 中断・再開パラメータ
        nh_.param("allow_gait_interruption", allow_gait_interruption_, true);
        nh_.param("min_step_completion", min_step_completion_, 0.3);
        
        // 静止判定パラメータ（応答性向上）
        nh_.param("zero_velocity_threshold", zero_velocity_threshold_, 0.001);  // m/s
        nh_.param("idle_timeout", idle_timeout_, 0.15);           // 0.15秒で停止開始（高速化）
        nh_.param("deceleration_time", deceleration_time_, 0.4);  // 0.4秒かけて減速（高速化）
        nh_.param("startup_blend_time", startup_blend_time_, 0.5); // 起動時0.5秒でブレンド
        
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
                 smooth_transition_.direction_change_rate, 
                 smooth_transition_.speed_change_rate,
                 smooth_transition_.angular_change_rate);
        ROS_INFO("  Stop parameters: idle_timeout=%.2fs, deceleration_time=%.2fs",
                 idle_timeout_, deceleration_time_);
        ROS_INFO("  Startup blend time: %.2fs", startup_blend_time_);
    }
    
    void setupHexapodConfiguration() {
        // 6脚の設定
        leg_names_ = {"RF", "LF", "LM", "LB", "RB", "RM"};
        std::vector<double> leg_angles = {0, 300, 240, 180, 120, 60};
        std::vector<int> tripod_groups = {0, 1, 0, 1, 0, 1};
        
        for (size_t i = 0; i < leg_names_.size(); ++i) {
            LegConfig config;
            config.leg_id = leg_names_[i];
            config.attach_angle = leg_angles[i];
            config.tripod_group = tripod_groups[i];
            config.phase_offset = (tripod_groups[i] == 0) ? 0.0 : 0.5;
            
            // ホームポジション設定
            config.home_x = 140.0;  // mm
            config.home_y = 0.0;
            config.home_z = -85.0;
            
            config.current_x = config.home_x;
            config.current_y = config.home_y;
            config.current_z = config.home_z;
            config.stop_x = config.home_x;
            config.stop_y = config.home_y;
            config.stop_z = config.home_z;
            config.step_progress = 0.0;
            config.was_walking = false;
            
            legs_[config.leg_id] = config;
        }
        
        ROS_INFO("Hexapod configuration completed - 6 legs in 2 tripod groups");
        ROS_INFO("  Group 0: RF(0°), LM(240°), RB(120°)");
        ROS_INFO("  Group 1: LF(300°), LB(180°), RM(60°)");
    }
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        if (emergency_stop_ || !is_enabled_) return;
        
        last_cmd_time_ = ros::Time::now().toSec();
        
        // 入力値から速度と方向を計算
        double linear_x_ms = msg->linear.x;  // m/s
        double linear_y_ms = msg->linear.y;  // m/s
        double angular_z = msg->angular.z;   // rad/s
        
        double linear_speed_ms = sqrt(linear_x_ms * linear_x_ms + linear_y_ms * linear_y_ms);
        double linear_speed_mm = linear_speed_ms * 1000.0;  // mm/s
        
        // 静止判定（閾値以下なら0とみなす）
        if (linear_speed_ms < zero_velocity_threshold_ && std::abs(angular_z) < 0.01) {
            // 停止を要求（滑らかに減速）
            smooth_transition_.target_speed = 0.0;
            smooth_transition_.target_angular = 0.0;
            
            if (!is_stopping_ && is_walking_) {
                is_stopping_ = true;
                stop_start_time_ = ros::Time::now().toSec();
                
                // 停止開始時の位置を記録
                for (auto& leg_pair : legs_) {
                    LegConfig& config = leg_pair.second;
                    config.stop_x = config.current_x;
                    config.stop_y = config.current_y;
                    config.stop_z = config.current_z;
                }
                
                ROS_INFO("Zero velocity detected - starting smooth deceleration");
            }
            return;
        }
        
        // 有効な入力がある場合、停止をキャンセル
        if (is_stopping_) {
            is_stopping_ = false;
            is_completely_stopped_ = false;
            ROS_INFO("Motion resumed - canceling stop");
        }
        
        // 方向計算
        double direction = atan2(linear_y_ms, linear_x_ms) * 180.0 / M_PI;  // 度
        
        // 目標値設定
        smooth_transition_.target_speed = linear_speed_mm;
        smooth_transition_.target_direction = direction;
        smooth_transition_.target_angular = angular_z;
        
        // 歩行開始判定
        if (!is_walking_ && (linear_speed_mm > 1.0 || std::abs(angular_z) > 0.01)) {
            startWalking();
        }
        
        // 停止状態フラグをクリア
        is_completely_stopped_ = false;
    }
    
    void enableCallback(const std_msgs::Bool::ConstPtr& msg) {
        is_enabled_ = msg->data;
        
        if (!is_enabled_) {
            // 無効化時は即座に完全停止
            forceStopAtCurrentPosition();
        }
        
        ROS_INFO("Hexapod %s", is_enabled_ ? "ENABLED" : "DISABLED");
    }
    
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data) {
            emergency_stop_ = true;
            is_enabled_ = false;
            is_walking_ = false;
            is_completely_stopped_ = true;
            is_stopping_ = false;
            
            // 即座に現在位置で静止
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
        if (is_walking_) return;  // 既に歩行中なら何もしない
        
        is_walking_ = true;
        is_completely_stopped_ = false;
        is_stopping_ = false;
        current_time_ = 0.0;
        walk_start_time_ = ros::Time::now().toSec();
        
        // 各脚の歩行開始時刻と初期位置を記録
        for (auto& leg_pair : legs_) {
            LegConfig& config = leg_pair.second;
            leg_step_start_time_[config.leg_id] = current_time_;
            config.was_walking = true;
            
            // 停止時の位置を記録（ブレンド用）
            config.stop_x = config.current_x;
            config.stop_y = config.current_y;
            config.stop_z = config.current_z;
        }
        
        ROS_INFO("Walking started - Smooth startup blend enabled");
    }
    
    void forceStopAtCurrentPosition() {
        // 現在位置で即座に停止（緊急停止用）
        is_walking_ = false;
        is_completely_stopped_ = true;
        is_stopping_ = false;
        
        smooth_transition_.target_speed = 0.0;
        smooth_transition_.target_angular = 0.0;
        smooth_transition_.current_speed = 0.0;
        smooth_transition_.current_angular = 0.0;
        
        // 現在位置で固定
        for (auto& leg_pair : legs_) {
            LegConfig& config = leg_pair.second;
            
            dual_leg_controller::LegPosition cmd;
            cmd.x = config.current_x;
            cmd.y = config.current_y;
            cmd.z = config.current_z;
            
            config.was_walking = false;
            leg_pos_pubs_[config.leg_id].publish(cmd);
        }
        
        ROS_INFO("Forced stop at current position");
    }
    
    void controlCallback(const ros::TimerEvent& event) {
        if (emergency_stop_) return;
        
        current_time_ += 0.02; // 50Hz
        
        // タイムアウトチェック
        if (is_walking_ && !is_stopping_ && (ros::Time::now().toSec() - last_cmd_time_) > idle_timeout_) {
            is_stopping_ = true;
            stop_start_time_ = ros::Time::now().toSec();
            smooth_transition_.target_speed = 0.0;
            smooth_transition_.target_angular = 0.0;
            
            // 停止開始時の位置を記録
            for (auto& leg_pair : legs_) {
                LegConfig& config = leg_pair.second;
                config.stop_x = config.current_x;
                config.stop_y = config.current_y;
                config.stop_z = config.current_z;
            }
            
            ROS_INFO("Idle timeout - starting smooth deceleration");
        }
        
        // 完全停止状態の場合は更新しない
        if (is_completely_stopped_) {
            return;
        }
        
        // 停止中の処理
        if (is_stopping_) {
            double stop_elapsed = ros::Time::now().toSec() - stop_start_time_;
            
            if (stop_elapsed >= deceleration_time_) {
                // 減速完了 - 現在位置で完全停止
                is_walking_ = false;
                is_completely_stopped_ = true;
                is_stopping_ = false;
                
                smooth_transition_.current_speed = 0.0;
                smooth_transition_.current_angular = 0.0;
                
                // 現在位置で足を固定
                for (auto& leg_pair : legs_) {
                    LegConfig& config = leg_pair.second;
                    
                    dual_leg_controller::LegPosition cmd;
                    cmd.x = config.current_x;
                    cmd.y = config.current_y;
                    cmd.z = config.current_z;
                    
                    config.was_walking = false;
                    leg_pos_pubs_[config.leg_id].publish(cmd);
                }
                
                ROS_INFO("Smooth stop completed - holding position");
                return;
            }
        }
        
        // 通常のスムーズな遷移処理
        updateSmoothTransition();
        
        // 歩行制御（有効かつ歩行中のみ）
        if (is_walking_ && is_enabled_) {
            updateWalkingGait();
        }
    }
    
    void updateSmoothTransition() {
        double dt = 0.02; // 50Hz
        
        // 停止中は特別な減速処理
        if (is_stopping_) {
            double stop_elapsed = ros::Time::now().toSec() - stop_start_time_;
            double progress = std::min(1.0, stop_elapsed / deceleration_time_);
            
            // スムーズな減速カーブ（easeOutQuad）
            double decel_factor = 1.0 - progress * progress;
            
            smooth_transition_.current_speed *= decel_factor;
            smooth_transition_.current_angular *= decel_factor;
        } else {
            // 通常の加速・減速処理
            
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
            
            // 現在方向の正規化（0°〜360°）
            while (smooth_transition_.current_direction < 0.0) smooth_transition_.current_direction += 360.0;
            while (smooth_transition_.current_direction >= 360.0) smooth_transition_.current_direction -= 360.0;
            
            // 角速度の滑らかな変更
            double angular_diff = smooth_transition_.target_angular - smooth_transition_.current_angular;
            double max_angular_change = smooth_transition_.angular_change_rate * dt;
            
            if (std::abs(angular_diff) > max_angular_change) {
                smooth_transition_.current_angular += (angular_diff > 0 ? max_angular_change : -max_angular_change);
            } else {
                smooth_transition_.current_angular = smooth_transition_.target_angular;
            }
        }
        
        // ロボット速度コマンドに変換
        double direction_rad = smooth_transition_.current_direction * M_PI / 180.0;
        robot_cmd_.velocity_x = smooth_transition_.current_speed * cos(direction_rad);
        robot_cmd_.velocity_y = smooth_transition_.current_speed * sin(direction_rad);
        robot_cmd_.angular_z = smooth_transition_.current_angular;
    }
    
    void updateWalkingGait() {
        // 歩行開始からの経過時間
        double walk_elapsed = ros::Time::now().toSec() - walk_start_time_;
        double startup_blend = std::min(1.0, walk_elapsed / startup_blend_time_);
        
        for (auto& leg_pair : legs_) {
            LegConfig& config = leg_pair.second;
            
            // 各脚の位相計算
            double phase = fmod((current_time_ / walk_params_.cycle_time) + config.phase_offset, 1.0);
            
            // 歩行パターンでの足先位置計算
            double walk_x, walk_y, walk_z;
            calculateLegFootPosition(config, phase, walk_x, walk_y, walk_z);
            
            double final_x, final_y, final_z;
            
            if (is_stopping_) {
                // 停止中：停止開始位置に向かって補間
                double stop_elapsed = ros::Time::now().toSec() - stop_start_time_;
                double stop_progress = std::min(1.0, stop_elapsed / deceleration_time_);
                
                // スムーズな停止カーブ（easeOutCubic）
                double stop_blend = 1.0 - pow(1.0 - stop_progress, 3.0);
                
                final_x = walk_x * (1.0 - stop_blend) + config.stop_x * stop_blend;
                final_y = walk_y * (1.0 - stop_blend) + config.stop_y * stop_blend;
                final_z = walk_z * (1.0 - stop_blend) + config.stop_z * stop_blend;
            } else if (startup_blend < 1.0) {
                // 起動時：停止位置から歩行パターンへスムーズにブレンド
                // easeInOutQuad カーブを使用
                double blend;
                if (startup_blend < 0.5) {
                    blend = 2.0 * startup_blend * startup_blend;
                } else {
                    blend = 1.0 - pow(-2.0 * startup_blend + 2.0, 2.0) / 2.0;
                }
                
                final_x = config.stop_x * (1.0 - blend) + walk_x * blend;
                final_y = config.stop_y * (1.0 - blend) + walk_y * blend;
                final_z = config.stop_z * (1.0 - blend) + walk_z * blend;
            } else {
                // 通常歩行
                final_x = walk_x;
                final_y = walk_y;
                final_z = walk_z;
            }
            
            // コマンド送信
            dual_leg_controller::LegPosition cmd;
            cmd.x = final_x;
            cmd.y = final_y;
            cmd.z = final_z;
            
            config.current_x = final_x;
            config.current_y = final_y;
            config.current_z = final_z;
            
            leg_pos_pubs_[config.leg_id].publish(cmd);
        }
    }
    
    void calculateLegFootPosition(const LegConfig& config, double phase, 
                                   double& x, double& y, double& z) {
        // 脚の取り付け角度をラジアンに変換
        double attach_rad = config.attach_angle * M_PI / 180.0;
        
        // ロボット座標系での速度を脚座標系に変換
        double leg_velocity_x = robot_cmd_.velocity_x * cos(attach_rad) + 
                                robot_cmd_.velocity_y * sin(attach_rad);
        double leg_velocity_y = -robot_cmd_.velocity_x * sin(attach_rad) + 
                                robot_cmd_.velocity_y * cos(attach_rad);
        
        // 回転による速度成分を追加（脚の取り付け位置からの距離は140mm）
        double rotation_radius = 140.0;  // mm
        double tangential_velocity = robot_cmd_.angular_z * rotation_radius;  // mm/s
        
        // 回転方向は脚の取り付け角度に対して90度
        double rotation_velocity_x = -tangential_velocity * sin(attach_rad);
        double rotation_velocity_y = tangential_velocity * cos(attach_rad);
        
        leg_velocity_x += rotation_velocity_x;
        leg_velocity_y += rotation_velocity_y;
        
        // 速度から歩幅を計算
        double velocity_magnitude = sqrt(leg_velocity_x * leg_velocity_x + 
                                        leg_velocity_y * leg_velocity_y);
        
        double leg_step_x, leg_step_y;
        if (velocity_magnitude > 0.1) {
            leg_step_x = (leg_velocity_x / velocity_magnitude) * walk_params_.step_length;
            leg_step_y = (leg_velocity_y / velocity_magnitude) * walk_params_.step_length;
        } else {
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
            
            // 放物線軌道
            double swing_height = 4.0 * walk_params_.step_height * step_progress * (1.0 - step_progress);
            z_offset = swing_height;
        } else {
            // スタンス期：後方移動（地面接触）
            step_progress = (phase - 0.5) * 2.0;  // 0→1
            x_offset = leg_step_x * 0.5 - leg_step_x * step_progress;
            y_offset = leg_step_y * 0.5 - leg_step_y * step_progress;
            z_offset = 0.0;
        }
        
        // 最終的な足先位置
        x = config.home_x + x_offset;
        y = config.home_y + y_offset;
        z = config.home_z + z_offset;
    }
    
    void moveToHome() {
        is_walking_ = false;
        is_enabled_ = false;
        is_completely_stopped_ = true;
        is_stopping_ = false;
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
            config.stop_x = cmd.x;
            config.stop_y = cmd.y;
            config.stop_z = cmd.z;
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
        ROS_INFO("  ✓ 実績ある足先計算ロジック使用");
        ROS_INFO("  ✓ 速度・方向・回転の滑らかな遷移");
        ROS_INFO("  ✓ 待機時完全静止機能（足踏みなし）");
        ROS_INFO("  ✓ スムーズな起動・停止（ブレンド機能）");
        ROS_INFO("  ✓ 高応答性（素早い停止）");
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

// 滑らかな方向変更対応6脚歩行制御システム
#include <ros/ros.h>
#include "dual_leg_controller/LegPosition.h"
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <map>
#include <vector>

class SmoothHexapodDirectionalController {
private:
    ros::NodeHandle nh_;
    std::map<std::string, ros::Publisher> leg_pos_pubs_;
    ros::Subscriber cmd_vel_sub_;
    ros::Timer control_timer_;
    
    // 脚の設定情報
    struct LegConfig {
        std::string leg_id;
        int leg_index;
        double attach_angle;
        double home_x, home_y, home_z;
        bool enabled;
        int tripod_group;
        
        // 現在の状態（滑らかな遷移用）
        double current_x, current_y, current_z;  // 現在の実際の位置
        double last_phase;                       // 前回の位相
        bool was_walking;                        // 前回歩行していたか
    };
    
    std::map<std::string, LegConfig> legs_;
    std::vector<std::string> leg_names_;
    
    // 歩行パラメータ
    struct WalkParams {
        double step_height = 17.0;
        double step_length = 60.0;
        double cycle_time = 1.2;
        double speed_multiplier = 1.0;
    } walk_params_;
    
    // 滑らかな方向変更用
    struct SmoothTransition {
        double target_direction = 0.0;     // 目標方向
        double current_direction = 0.0;    // 現在の方向
        double direction_change_rate = 90.0; // 方向変更速度 [度/秒]
        
        double target_speed = 20.0;        // 目標速度
        double current_speed = 20.0;       // 現在の速度
        double speed_change_rate = 50.0;   // 速度変更率 [mm/s^2]
        
        double target_angular = 0.0;       // 目標角速度
        double current_angular = 0.0;      // 現在の角速度
        double angular_change_rate = 1.0;  // 角速度変更率 [rad/s^2]
    } smooth_transition_;
    
    // ロボット制御状態
    struct RobotCommand {
        double velocity_x = 0.0;
        double velocity_y = 0.0;
        double angular_z = 0.0;
        double walk_direction = 0.0;
        double walk_speed = 20.0;
    } robot_cmd_;
    
    // 制御状態
    bool is_walking_;
    bool is_enabled_;
    double current_time_;
    bool is_transitioning_;  // 遷移中かどうか
    
    // キーボード制御用
    struct termios old_termios_;
    bool keyboard_initialized_;

public:
    SmoothHexapodDirectionalController() : nh_(""), is_walking_(false), is_enabled_(false), 
                                          current_time_(0.0), keyboard_initialized_(false),
                                          is_transitioning_(false) {
        
        setupHexapodConfiguration();
        
        // トピック設定
        for (const auto& leg_pair : legs_) {
            const std::string& leg_id = leg_pair.first;
            std::string topic = "/asterisk/leg/" + leg_id + "/command/foot_position";
            leg_pos_pubs_[leg_id] = nh_.advertise<dual_leg_controller::LegPosition>(topic, 1);
        }
        
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, 
                                   &SmoothHexapodDirectionalController::cmdVelCallback, this);
        
        // 制御タイマー（50Hz）
        control_timer_ = nh_.createTimer(ros::Duration(0.02), 
                                       &SmoothHexapodDirectionalController::controlCallback, this);
        
        initializeKeyboard();
        
        ROS_INFO("Smooth Hexapod Directional Controller initialized");
        printInstructions();
        moveToHome();
    }
    
    ~SmoothHexapodDirectionalController() {
        if (keyboard_initialized_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
        }
    }

private:
    void setupHexapodConfiguration() {
        leg_names_ = {"RF", "LF", "LM", "LB", "RB", "RM"};
        double attach_angles[6] = {0, 300, 240, 180, 120, 60};
        int tripod_groups[6] = {0, 1, 0, 1, 0, 1};
        
        for (int i = 0; i < 6; i++) {
            legs_[leg_names_[i]] = {
                leg_names_[i],
                i,
                attach_angles[i],
                140.0, 0.0, -90.0,
                true,
                tripod_groups[i],
                140.0, 0.0, -90.0,  // 現在位置をホーム位置で初期化
                0.0,                 // 初期位相
                false               // 初期状態では歩行していない
            };
        }
        
        // 滑らかな遷移パラメータの初期化
        smooth_transition_.current_direction = robot_cmd_.walk_direction;
        smooth_transition_.current_speed = robot_cmd_.walk_speed;
        smooth_transition_.current_angular = robot_cmd_.angular_z;
        
        ROS_INFO("Smooth Hexapod Configuration initialized");
    }
    
    void controlCallback(const ros::TimerEvent&) {
        handleKeyboardInput();
        
        // 滑らかな遷移の更新
        updateSmoothTransition();
        
        if (is_walking_ && is_enabled_) {
            updateSmoothDirectionalWalking();
        } else if (is_enabled_) {
            // 歩行停止時も現在位置を維持
            maintainCurrentPositions();
        }
    }
    
    void updateSmoothTransition() {
        double dt = 0.02;  // 50Hz
        
        // 方向の滑らかな変更
        double direction_diff = smooth_transition_.target_direction - smooth_transition_.current_direction;
        
        // 角度の正規化（-180°〜180°）
        while (direction_diff > 180.0) direction_diff -= 360.0;
        while (direction_diff < -180.0) direction_diff += 360.0;
        
        double max_direction_change = smooth_transition_.direction_change_rate * dt;
        
        if (fabs(direction_diff) > max_direction_change) {
            if (direction_diff > 0) {
                smooth_transition_.current_direction += max_direction_change;
            } else {
                smooth_transition_.current_direction -= max_direction_change;
            }
            is_transitioning_ = true;
        } else {
            smooth_transition_.current_direction = smooth_transition_.target_direction;
            is_transitioning_ = false;
        }
        
        // 角度の正規化（0°〜360°）
        while (smooth_transition_.current_direction < 0.0) {
            smooth_transition_.current_direction += 360.0;
        }
        while (smooth_transition_.current_direction >= 360.0) {
            smooth_transition_.current_direction -= 360.0;
        }
        
        // 速度の滑らかな変更
        double speed_diff = smooth_transition_.target_speed - smooth_transition_.current_speed;
        double max_speed_change = smooth_transition_.speed_change_rate * dt;
        
        if (fabs(speed_diff) > max_speed_change) {
            if (speed_diff > 0) {
                smooth_transition_.current_speed += max_speed_change;
            } else {
                smooth_transition_.current_speed -= max_speed_change;
            }
        } else {
            smooth_transition_.current_speed = smooth_transition_.target_speed;
        }
        
        // 角速度の滑らかな変更
        double angular_diff = smooth_transition_.target_angular - smooth_transition_.current_angular;
        double max_angular_change = smooth_transition_.angular_change_rate * dt;
        
        if (fabs(angular_diff) > max_angular_change) {
            if (angular_diff > 0) {
                smooth_transition_.current_angular += max_angular_change;
            } else {
                smooth_transition_.current_angular -= max_angular_change;
            }
        } else {
            smooth_transition_.current_angular = smooth_transition_.target_angular;
        }
        
        // ロボット指令の更新
        robot_cmd_.walk_direction = smooth_transition_.current_direction;
        robot_cmd_.walk_speed = smooth_transition_.current_speed;
        robot_cmd_.angular_z = smooth_transition_.current_angular;
        
        // 速度ベクトルの計算
        double walk_direction_rad = robot_cmd_.walk_direction * M_PI / 180.0;
        robot_cmd_.velocity_x = robot_cmd_.walk_speed * cos(walk_direction_rad);
        robot_cmd_.velocity_y = robot_cmd_.walk_speed * sin(walk_direction_rad);
    }
    
    void updateSmoothDirectionalWalking() {
        current_time_ += 0.02 * walk_params_.speed_multiplier;
        double effective_cycle_time = walk_params_.cycle_time / walk_params_.speed_multiplier;
        
        for (auto& leg_pair : legs_) {
            LegConfig& config = leg_pair.second;
            
            if (!config.enabled) continue;
            
            processSmoothLegMotion(config, effective_cycle_time);
        }
    }
    
    void processSmoothLegMotion(LegConfig& config, double effective_cycle_time) {
        // 現在の位相計算
        double phase_offset = (config.tripod_group == 0) ? 0.0 : 0.5;
        double new_phase = fmod(current_time_ / effective_cycle_time + phase_offset, 1.0);
        
        // 座標変換
        double attach_angle_rad = config.attach_angle * M_PI / 180.0;
        double leg_velocity_x =  robot_cmd_.velocity_x * cos(attach_angle_rad) + 
                                robot_cmd_.velocity_y * sin(attach_angle_rad);
        double leg_velocity_y = -robot_cmd_.velocity_x * sin(attach_angle_rad) + 
                                robot_cmd_.velocity_y * cos(attach_angle_rad);
        
        // 回転成分の追加
        double body_radius = 95.0;
        double rotation_velocity_x = -robot_cmd_.angular_z * body_radius * sin(attach_angle_rad);
        double rotation_velocity_y =  robot_cmd_.angular_z * body_radius * cos(attach_angle_rad);
        
        leg_velocity_x += rotation_velocity_x;
        leg_velocity_y += rotation_velocity_y;
        
        // ステップ幅計算
        double velocity_magnitude = sqrt(leg_velocity_x * leg_velocity_x + leg_velocity_y * leg_velocity_y);
        double leg_step_x, leg_step_y;
        
        if (velocity_magnitude > 0.1) {
            leg_step_x = (leg_velocity_x / velocity_magnitude) * walk_params_.step_length;
            leg_step_y = (leg_velocity_y / velocity_magnitude) * walk_params_.step_length;
        } else {
            leg_step_x = 0.0;
            leg_step_y = 0.0;
        }
        
        dual_leg_controller::LegPosition cmd;
        
        // 歩行開始時または方向変更時の滑らかな遷移
        if (!config.was_walking || is_transitioning_) {
            // 現在位置から滑らかに歩行パターンに移行
            double transition_factor = calculateTransitionFactor(config);
            
            // 目標歩行位置の計算
            double target_x, target_y, target_z;
            calculateWalkingPosition(new_phase, leg_step_x, leg_step_y, config, 
                                   target_x, target_y, target_z);
            
            // 現在位置から目標位置への補間
            cmd.x = config.current_x + (target_x - config.current_x) * transition_factor;
            cmd.y = config.current_y + (target_y - config.current_y) * transition_factor;
            cmd.z = config.current_z + (target_z - config.current_z) * transition_factor;
            
        } else {
            // 通常の歩行パターン
            calculateWalkingPosition(new_phase, leg_step_x, leg_step_y, config, 
                                   cmd.x, cmd.y, cmd.z);
        }
        
        // 位置の更新
        config.current_x = cmd.x;
        config.current_y = cmd.y;
        config.current_z = cmd.z;
        config.last_phase = new_phase;
        config.was_walking = true;
        
        leg_pos_pubs_[config.leg_id].publish(cmd);
    }
    
    double calculateTransitionFactor(const LegConfig& config) {
        // 遷移の滑らかさを制御（0.0から1.0）
        // スタンス期（地面についている時）により積極的に遷移
        double phase_offset = (config.tripod_group == 0) ? 0.0 : 0.5;
        double effective_cycle_time = walk_params_.cycle_time / walk_params_.speed_multiplier;
        double current_phase = fmod(current_time_ / effective_cycle_time + phase_offset, 1.0);
        
        if (current_phase >= 0.5) {  // スタンス期
            return std::min(1.0, (current_time_ * 2.0));  // より速い遷移
        } else {  // スイング期
            return std::min(1.0, (current_time_ * 1.0));  // 穏やかな遷移
        }
    }
    
    void calculateWalkingPosition(double phase, double leg_step_x, double leg_step_y, 
                                const LegConfig& config, double& x, double& y, double& z) {
        bool is_swing_phase = (phase < 0.5);
        double step_progress, x_offset, y_offset, z_offset;
        
        if (is_swing_phase) {
            // スイング期：前方移動（空中）
            step_progress = phase * 2.0;
            x_offset = -leg_step_x * 0.5 + leg_step_x * step_progress;
            y_offset = -leg_step_y * 0.5 + leg_step_y * step_progress;
            double swing_height = 4.0 * walk_params_.step_height * step_progress * (1.0 - step_progress);
            z_offset = swing_height;
        } else {
            // スタンス期：後方移動（地面接触）
            step_progress = (phase - 0.5) * 2.0;
            x_offset = leg_step_x * 0.5 - leg_step_x * step_progress;
            y_offset = leg_step_y * 0.5 - leg_step_y * step_progress;
            z_offset = 0.0;
        }
        
        x = config.home_x + x_offset;
        y = config.home_y + y_offset;
        z = config.home_z + z_offset;
    }
    
    void maintainCurrentPositions() {
        // 歩行していない時は現在位置を維持
        for (auto& leg_pair : legs_) {
            LegConfig& config = leg_pair.second;
            
            if (!config.enabled) continue;
            
            dual_leg_controller::LegPosition cmd;
            cmd.x = config.current_x;
            cmd.y = config.current_y;
            cmd.z = config.current_z;
            
            leg_pos_pubs_[config.leg_id].publish(cmd);
        }
    }
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        robot_cmd_.velocity_x = msg->linear.x * 1000.0;
        robot_cmd_.velocity_y = msg->linear.y * 1000.0;
        robot_cmd_.angular_z = msg->angular.z;
        
        double speed = sqrt(robot_cmd_.velocity_x * robot_cmd_.velocity_x + 
                           robot_cmd_.velocity_y * robot_cmd_.velocity_y);
        if (speed > 0.1) {
            smooth_transition_.target_direction = atan2(robot_cmd_.velocity_y, robot_cmd_.velocity_x) * 180.0 / M_PI;
            smooth_transition_.target_speed = speed;
        }
        smooth_transition_.target_angular = robot_cmd_.angular_z;
    }
    
    void printInstructions() {
        ROS_INFO("=== SMOOTH HEXAPOD DIRECTIONAL WALKING CONTROLLER ===");
        ROS_INFO("滑らかな方向変更対応6脚歩行システム");
        ROS_INFO(" ");
        ROS_INFO("IMPROVEMENTS:");
        ROS_INFO("  ✓ 滑らかな方向変更（%.1f度/秒）", smooth_transition_.direction_change_rate);
        ROS_INFO("  ✓ 現在位置からの滑らかな歩行開始");
        ROS_INFO("  ✓ 速度変更の平滑化（%.1fmm/s²）", smooth_transition_.speed_change_rate);
        ROS_INFO(" ");
        ROS_INFO("MAIN CONTROLS:");
        ROS_INFO("  SPACE: Start/Stop walking");
        ROS_INFO("  h: Home position");
        ROS_INFO("  r: Show current status");
        ROS_INFO("  ESC: Emergency stop and exit");
        ROS_INFO(" ");
        ROS_INFO("DIRECTION CONTROLS:");
        ROS_INFO("  w: Forward (0°)    s: Backward (180°)");
        ROS_INFO("  a: Left (90°)      d: Right (270°)");
        ROS_INFO("  z: F-Left (45°)    c: F-Right (315°)");
        ROS_INFO("  x: B-Left (135°)   v: B-Right (225°)");
        ROS_INFO(" ");
        ROS_INFO("SMOOTH ADJUSTMENTS:");
        ROS_INFO("  p/;: Direction change rate +/-30°/s");
        ROS_INFO("  [/]: Speed change rate +/-25mm/s²");
        ROS_INFO(" ");
        printCurrentStatus();
    }
    
    void printCurrentStatus() {
        ROS_INFO("=== CURRENT STATUS ===");
        ROS_INFO("Target Direction: %.1f° → Current: %.1f°", 
                 smooth_transition_.target_direction, smooth_transition_.current_direction);
        ROS_INFO("Target Speed: %.1fmm/s → Current: %.1fmm/s", 
                 smooth_transition_.target_speed, smooth_transition_.current_speed);
        ROS_INFO("Direction change rate: %.1f°/s", smooth_transition_.direction_change_rate);
        ROS_INFO("Speed change rate: %.1fmm/s²", smooth_transition_.speed_change_rate);
        if (is_transitioning_) {
            ROS_INFO("Status: TRANSITIONING 🔄");
        } else {
            ROS_INFO("Status: STABLE ✓");
        }
        ROS_INFO("=====================");
    }
    
    void handleKeyboardInput() {
        char key = getKeyPress();
        if (key != 0) {
            handleKeyInput(key);
        }
    }
    
    void handleKeyInput(char key) {
        switch (key) {
            case ' ':
                toggleWalking();
                break;
                
            // 8方向移動（滑らかに遷移）
            case 'w':
                smooth_transition_.target_direction = 0.0;
                printCurrentStatus();
                break;
            case 's':
                smooth_transition_.target_direction = 180.0;
                printCurrentStatus();
                break;
            case 'a':
                smooth_transition_.target_direction = 90.0;
                printCurrentStatus();
                break;
            case 'd':
                smooth_transition_.target_direction = 270.0;
                printCurrentStatus();
                break;
            case 'z':
                smooth_transition_.target_direction = 45.0;
                printCurrentStatus();
                break;
            case 'c':
                smooth_transition_.target_direction = 315.0;
                printCurrentStatus();
                break;
            case 'x':
                smooth_transition_.target_direction = 135.0;
                printCurrentStatus();
                break;
            case 'v':
                smooth_transition_.target_direction = 225.0;
                printCurrentStatus();
                break;
                
            // 速度制御
            case 't':
                smooth_transition_.target_speed = std::min(60.0, smooth_transition_.target_speed + 5.0);
                printCurrentStatus();
                break;
            case 'g':
                smooth_transition_.target_speed = std::max(5.0, smooth_transition_.target_speed - 5.0);
                printCurrentStatus();
                break;
                
            // 回転制御
            case 'q':
                smooth_transition_.target_angular = std::min(0.8, smooth_transition_.target_angular + 0.2);
                printCurrentStatus();
                break;
            case 'e':
                smooth_transition_.target_angular = std::max(-0.8, smooth_transition_.target_angular - 0.2);
                printCurrentStatus();
                break;
                
            // 滑らかさの調整
            case 'p':
                smooth_transition_.direction_change_rate = std::min(180.0, smooth_transition_.direction_change_rate + 30.0);
                ROS_INFO("Direction change rate: %.1f°/s", smooth_transition_.direction_change_rate);
                break;
            case ';':
                smooth_transition_.direction_change_rate = std::max(30.0, smooth_transition_.direction_change_rate - 30.0);
                ROS_INFO("Direction change rate: %.1f°/s", smooth_transition_.direction_change_rate);
                break;
                
            case '[':
                smooth_transition_.speed_change_rate = std::max(25.0, smooth_transition_.speed_change_rate - 25.0);
                ROS_INFO("Speed change rate: %.1fmm/s²", smooth_transition_.speed_change_rate);
                break;
            case ']':
                smooth_transition_.speed_change_rate = std::min(200.0, smooth_transition_.speed_change_rate + 25.0);
                ROS_INFO("Speed change rate: %.1fmm/s²", smooth_transition_.speed_change_rate);
                break;
                
            case 'h':
                moveToHome();
                break;
            case 'r':
                printCurrentStatus();
                break;
            case 'n':
                smooth_transition_.target_speed = 0.0;
                smooth_transition_.target_angular = 0.0;
                printCurrentStatus();
                break;
                
            case 27:
                ROS_INFO("Emergency stop - Exiting...");
                moveToHome();
                ros::shutdown();
                break;
        }
    }
    
    void toggleWalking() {
        if (!is_enabled_) {
            is_enabled_ = true;
            ROS_INFO("All legs enabled");
            
            // 現在位置の初期化
            for (auto& leg_pair : legs_) {
                LegConfig& config = leg_pair.second;
                config.was_walking = false;  // 歩行開始時は滑らかに遷移
            }
        }
        
        is_walking_ = !is_walking_;
        if (is_walking_) {
            current_time_ = 0.0;
            ROS_INFO("Smooth hexapod walking started!");
            ROS_INFO("Target direction: %.1f° (%s), Speed: %.1fmm/s", 
                     smooth_transition_.target_direction, getDirectionName().c_str(), 
                     smooth_transition_.target_speed);
        } else {
            ROS_INFO("Walking stopped - maintaining current positions");
            
            // 歩行停止時は現在位置を維持
            for (auto& leg_pair : legs_) {
                LegConfig& config = leg_pair.second;
                config.was_walking = false;
            }
        }
    }
    
    std::string getDirectionName() {
        double dir = fmod(smooth_transition_.current_direction + 360.0, 360.0);
        
        if (dir >= 337.5 || dir < 22.5) return "Forward";
        else if (dir >= 22.5 && dir < 67.5) return "Forward-Right";
        else if (dir >= 67.5 && dir < 112.5) return "Right";
        else if (dir >= 112.5 && dir < 157.5) return "Backward-Right";
        else if (dir >= 157.5 && dir < 202.5) return "Backward";
        else if (dir >= 202.5 && dir < 247.5) return "Backward-Left";
        else if (dir >= 247.5 && dir < 292.5) return "Left";
        else if (dir >= 292.5 && dir < 337.5) return "Forward-Left";
        else return "Unknown";
    }
    
    void moveToHome() {
        is_walking_ = false;
        is_enabled_ = false;
        smooth_transition_.target_angular = 0.0;
        smooth_transition_.current_angular = 0.0;
        robot_cmd_.angular_z = 0.0;
        
        for (auto& leg_pair : legs_) {
            LegConfig& config = leg_pair.second;
            
            dual_leg_controller::LegPosition cmd;
            cmd.x = config.home_x;
            cmd.y = config.home_y;
            cmd.z = config.home_z;
            
            // 現在位置もホーム位置にリセット
            config.current_x = cmd.x;
            config.current_y = cmd.y;
            config.current_z = cmd.z;
            config.was_walking = false;
            
            leg_pos_pubs_[config.leg_id].publish(cmd);
        }
        
        ROS_INFO("All legs moved to home position");
    }
    
    void initializeKeyboard() {
        tcgetattr(STDIN_FILENO, &old_termios_);
        struct termios new_termios = old_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
        keyboard_initialized_ = true;
    }
    
    char getKeyPress() {
        char key;
        if (read(STDIN_FILENO, &key, 1) == 1) {
            return key;
        }
        return 0;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "smooth_hexapod_directional_controller");
    
    try {
        SmoothHexapodDirectionalController hexapod;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return 1;
    }
    
    return 0;
}
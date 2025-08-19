// 相対位置計算による前進歩行制御（RF-LF）
#include <ros/ros.h>
#include "dual_leg_controller/LegPosition.h"
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <map>

class RelativePositionWalk {
private:
    ros::NodeHandle nh_;
    std::map<std::string, ros::Publisher> leg_pos_pubs_;
    ros::Timer control_timer_;
    
    // 脚の設定情報
    struct LegConfig {
        std::string leg_id;
        double attach_angle;        // 取り付け角度 [rad]
        double home_x, home_y, home_z;  // ホーム位置（脚座標系）
        bool enabled;
        
        // ロボット移動用のパラメータ
        double robot_offset_x;      // ロボット移動量に対する脚座標系X方向の寄与
        double robot_offset_y;      // ロボット移動量に対する脚座標系Y方向の寄与
    };
    
    std::map<std::string, LegConfig> legs_;
    
    // 歩行パラメータ（検証済み値）
    struct WalkParams {
        double step_height = 17.0;       // [mm]
        double step_length = 60.0;       // [mm]
        double cycle_time = 1.2;         // [s]
        double speed_multiplier = 1.0;
    } walk_params_;
    
    // 制御状態
    bool is_walking_;
    bool is_enabled_;
    double current_time_;
    double robot_forward_distance_;     // ロボット全体の前進距離
    double forward_speed_;              // 前進速度 [mm/s]
    
    // キーボード制御用
    struct termios old_termios_;
    bool keyboard_initialized_;
    char last_key_;

public:
    RelativePositionWalk() : nh_(""), is_walking_(false), is_enabled_(false), 
                           current_time_(0.0), last_key_(0), keyboard_initialized_(false),
                           robot_forward_distance_(0.0), forward_speed_(20.0) {
        
        // 脚の設定（実際のロボット構成に合わせて）
        setupLegConfigurations();
        
        // トピック設定
        for (const auto& leg_pair : legs_) {
            const std::string& leg_id = leg_pair.first;
            std::string topic = "/asterisk/leg/" + leg_id + "/command/foot_position";
            leg_pos_pubs_[leg_id] = nh_.advertise<dual_leg_controller::LegPosition>(topic, 1);
        }
        
        // 制御タイマー（50Hz）
        control_timer_ = nh_.createTimer(ros::Duration(0.02), 
                                       &RelativePositionWalk::controlCallback, this);
        
        // キーボード初期化
        initializeKeyboard();
        
        ROS_INFO("Relative Position Walk Controller initialized");
        printInstructions();
        moveToHome();
    }
    
    ~RelativePositionWalk() {
        if (keyboard_initialized_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
        }
    }

private:
    void setupLegConfigurations() {
        // 脚の取り付け角度（ラジアン）と相対位置計算
        double body_radius = 95.0;  // [mm]
        
        // RF脚: 0度位置に取り付け
        legs_["RF"] = {
            "RF",
            0.0,                    // 取り付け角度 0度
            140.0, 0.0, -90.0,     // ホーム位置（脚座標系）
            true,
            1.0, 0.0               // 前進時: 脚座標系X方向に寄与
        };
        
        // LF脚: 60度位置に取り付け  
        legs_["LF"] = {
            "LF", 
            M_PI/3,                 // 取り付け角度 60度
            140.0, 0.0, -90.0,     // ホーム位置（脚座標系）
            true,
            0.5, -0.866            // 前進時: cos(60°), -sin(60°)の寄与
        };
        
        ROS_INFO("Leg configurations:");
        for (const auto& leg_pair : legs_) {
            const LegConfig& config = leg_pair.second;
            ROS_INFO("  %s: attach_angle=%.1f°, robot_offset=(%.3f, %.3f)", 
                     config.leg_id.c_str(), config.attach_angle * 180.0 / M_PI,
                     config.robot_offset_x, config.robot_offset_y);
        }
    }
    
    void controlCallback(const ros::TimerEvent&) {
        handleKeyboardInput();
        
        if (is_walking_ && is_enabled_) {
            updateRelativePositionWalking();
        }
    }
    
    void updateRelativePositionWalking() {
        current_time_ += 0.02 * walk_params_.speed_multiplier;
        
        // ロボット全体の前進距離を更新
        robot_forward_distance_ += forward_speed_ * 0.02 * walk_params_.speed_multiplier;
        
        double effective_cycle_time = walk_params_.cycle_time / walk_params_.speed_multiplier;
        
        for (const auto& leg_pair : legs_) {
            const std::string& leg_id = leg_pair.first;
            const LegConfig& config = leg_pair.second;
            
            if (!config.enabled) continue;
            
            // 位相計算（RF=0, LF=0.5で180度位相差）
            double phase_offset = (leg_id == "LF") ? 0.5 : 0.0;
            double phase = fmod(current_time_ / effective_cycle_time + phase_offset, 1.0);
            
            // ロボット前進量に対する各脚の必要な動作量を計算
            double robot_step_x = walk_params_.step_length;  // ロボット前進量
            
            // 各脚の脚座標系での必要な動作量
            double leg_step_x = robot_step_x * config.robot_offset_x;
            double leg_step_y = robot_step_x * config.robot_offset_y;
            
            // スイング期/スタンス期の判定
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
                // スタンス期：後方移動（地面との接触）
                step_progress = (phase - 0.5) * 2.0;  // 0→1
                x_offset = leg_step_x * 0.5 - leg_step_x * step_progress;
                y_offset = leg_step_y * 0.5 - leg_step_y * step_progress;
                z_offset = 0.0;
            }
            
            // 最終的な足先位置（脚座標系）
            dual_leg_controller::LegPosition cmd;
            cmd.x = config.home_x + x_offset;
            cmd.y = config.home_y + y_offset;
            cmd.z = config.home_z + z_offset;
            
            leg_pos_pubs_[leg_id].publish(cmd);
        }
    }
    
    void printInstructions() {
        ROS_INFO("=== RELATIVE POSITION WALK TEST (RF-LF) ===");
        ROS_INFO("Leg offset calculations:");
        ROS_INFO("  RF: robot_offset=(%.3f, %.3f) - moves in leg X direction", 
                 legs_["RF"].robot_offset_x, legs_["RF"].robot_offset_y);
        ROS_INFO("  LF: robot_offset=(%.3f, %.3f) - moves at 60° to leg X", 
                 legs_["LF"].robot_offset_x, legs_["LF"].robot_offset_y);
        ROS_INFO("");
        ROS_INFO("CONTROLS:");
        ROS_INFO("  SPACE: Start/Continue walking (hold to walk)");
        ROS_INFO("  s: Stop and hold position");
        ROS_INFO("  h: Return to home position");
        ROS_INFO("  r: Reset robot position");
        ROS_INFO("  ESC/q: Emergency stop and exit");
        ROS_INFO("");
        ROS_INFO("ADJUSTMENTS:");
        ROS_INFO("  +/-: Step height (%.1fmm)", walk_params_.step_height);
        ROS_INFO("  [/]: Step length (%.1fmm)", walk_params_.step_length);
        ROS_INFO("  {/}: Cycle time (%.1fs)", walk_params_.cycle_time);
        ROS_INFO("  f/v: Forward speed (%.1fmm/s)", forward_speed_);
        ROS_INFO("");
        printCurrentParams();
    }
    
    void printCurrentParams() {
        ROS_INFO("Current: height=%.1fmm, length=%.1fmm, cycle=%.1fs, speed=%.1fmm/s, distance=%.1fmm", 
                 walk_params_.step_height, walk_params_.step_length, 
                 walk_params_.cycle_time, forward_speed_, robot_forward_distance_);
    }
    
    void handleKeyboardInput() {
        char key = getKeyPress();
        if (key != 0 && key != last_key_) {
            last_key_ = key;
            handleKeyInput(key);
        } else if (key == 0) {
            last_key_ = 0;
        }
    }
    
    void handleKeyInput(char key) {
        switch (key) {
            case ' ':  // 歩行開始/継続
                if (!is_walking_) {
                    startWalking();
                }
                break;
                
            case 's':  // 停止
                stopWalking();
                break;
                
            case 'h':  // ホームポジション
                moveToHome();
                break;
                
            case 'r':  // 位置リセット
                resetRobotPosition();
                break;
                
            case '+':  // ステップ高さ増加
                walk_params_.step_height = std::min(30.0, walk_params_.step_height + 1.0);
                printCurrentParams();
                break;
                
            case '-':  // ステップ高さ減少
                walk_params_.step_height = std::max(5.0, walk_params_.step_height - 1.0);
                printCurrentParams();
                break;
                
            case '[':  // ステップ長さ減少
                walk_params_.step_length = std::max(20.0, walk_params_.step_length - 5.0);
                printCurrentParams();
                break;
                
            case ']':  // ステップ長さ増加
                walk_params_.step_length = std::min(80.0, walk_params_.step_length + 5.0);
                printCurrentParams();
                break;
                
            case '{':  // サイクル時間増加（遅く）
                walk_params_.cycle_time = std::min(4.0, walk_params_.cycle_time + 0.1);
                printCurrentParams();
                break;
                
            case '}':  // サイクル時間減少（速く）
                walk_params_.cycle_time = std::max(0.8, walk_params_.cycle_time - 0.1);
                printCurrentParams();
                break;
                
            case 'f':  // 前進速度増加
                forward_speed_ = std::min(50.0, forward_speed_ + 5.0);
                printCurrentParams();
                break;
                
            case 'v':  // 前進速度減少
                forward_speed_ = std::max(5.0, forward_speed_ - 5.0);
                printCurrentParams();
                break;
                
            case 27:   // ESC
            case 'q':  // 終了
                ROS_INFO("Emergency stop - Exiting...");
                moveToHome();
                ros::shutdown();
                break;
                
            default:
                break;
        }
    }
    
    void startWalking() {
        if (!is_enabled_) {
            is_enabled_ = true;
            ROS_INFO("Legs enabled");
        }
        
        if (!is_walking_) {
            is_walking_ = true;
            current_time_ = 0.0;
            ROS_INFO("Walking started - Using relative position calculations");
        }
    }
    
    void stopWalking() {
        is_walking_ = false;
        ROS_INFO("Walking stopped");
    }
    
    void moveToHome() {
        is_walking_ = false;
        is_enabled_ = false;
        
        for (const auto& leg_pair : legs_) {
            const std::string& leg_id = leg_pair.first;
            const LegConfig& config = leg_pair.second;
            
            dual_leg_controller::LegPosition cmd;
            cmd.x = config.home_x;
            cmd.y = config.home_y; 
            cmd.z = config.home_z;
            
            leg_pos_pubs_[leg_id].publish(cmd);
        }
        
        ROS_INFO("Moved to home position");
    }
    
    void resetRobotPosition() {
        robot_forward_distance_ = 0.0;
        ROS_INFO("Robot position reset");
        printCurrentParams();
    }
    
    // キーボード制御関数
    void initializeKeyboard() {
        tcgetattr(STDIN_FILENO, &old_termios_);
        struct termios new_termios = old_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios_);
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
    ros::init(argc, argv, "relative_position_walk");
    
    try {
        RelativePositionWalk walker;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return 1;
    }
    
    return 0;
}
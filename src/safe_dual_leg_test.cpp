// 安全な2脚協調歩行テスト（RF-LF）
#include <ros/ros.h>
#include "dual_leg_controller/LegPosition.h"
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <map>

class SafeDualLegTest {
private:
    ros::NodeHandle nh_;
    std::map<std::string, ros::Publisher> leg_pos_pubs_;
    ros::Timer control_timer_;
    
    // 検証済みの歩行パラメータ
    struct WalkParams {
        double step_height = 15.0;       // 検証済み値
        double step_length = 25.0;       // 検証済み値  
        double cycle_time = 2.0;         // 検証済み値
        double speed_multiplier = 0.5;   // 検証済み値
        double phase_offset = 0.5;       // RF-LF間の位相差（180度）
    } walk_params_;
    
    // 脚の基準位置
    struct LegConfig {
        std::string leg_id;
        double home_x, home_y, home_z;
        bool active;
    };
    
    std::map<std::string, LegConfig> legs_;
    
    // 制御状態
    bool is_walking_;
    bool is_enabled_;
    double current_time_;
    char last_key_;
    
    // キーボード入力用
    struct termios old_tio_, new_tio_;
    bool keyboard_initialized_;

public:
    SafeDualLegTest() : nh_(""), is_walking_(false), is_enabled_(false), 
                       current_time_(0.0), last_key_(0), keyboard_initialized_(false) {
        
        // 脚の設定（RF, LF）
        legs_["RF"] = {"RF", 140.0, -50.0, -90.0, true};   // 右前
        legs_["LF"] = {"LF", 140.0, 50.0, -90.0, true};    // 左前
        
        // Publisher設定
        for (const auto& leg_pair : legs_) {
            const std::string& leg_id = leg_pair.first;
            leg_pos_pubs_[leg_id] = nh_.advertise<dual_leg_controller::LegPosition>(
                "/asterisk/leg/" + leg_id + "/command/foot_position", 1);
        }
        
        // タイマー設定（50Hz）
        control_timer_ = nh_.createTimer(ros::Duration(0.02), 
            &SafeDualLegTest::controlTimerCallback, this);
        
        // キーボード設定
        initializeKeyboard();
        
        // 最初はホームポジションに移動
        moveToHome();
        
        printInstructions();
    }
    
    ~SafeDualLegTest() {
        if (keyboard_initialized_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
        }
        moveToHome();
        ros::Duration(0.5).sleep();
    }
    
    void initializeKeyboard() {
        tcgetattr(STDIN_FILENO, &old_tio_);
        new_tio_ = old_tio_;
        new_tio_.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
        keyboard_initialized_ = true;
    }
    
    char getKey() {
        char key = 0;
        if (read(STDIN_FILENO, &key, 1) == 1) {
            return key;
        }
        return 0;
    }
    
    void printInstructions() {
        ROS_INFO("=== SAFE DUAL LEG WALK TEST (RF-LF) ===");
        ROS_INFO("Using optimized parameters from single leg test:");
        ROS_INFO("  Height: %.1fmm, Length: %.1fmm", walk_params_.step_height, walk_params_.step_length);
        ROS_INFO("  Cycle: %.1fs, Speed: x%.1f", walk_params_.cycle_time, walk_params_.speed_multiplier);
        ROS_INFO("  Phase offset: %.1f (180 degrees)", walk_params_.phase_offset);
        ROS_INFO("");
        ROS_INFO("CONTROLS:");
        ROS_INFO("  SPACE: Start/Continue walking (hold to walk)");
        ROS_INFO("  s: Stop walking");
        ROS_INFO("  h: Return to home position");
        ROS_INFO("  ESC/q: Emergency stop and exit");
        ROS_INFO("");
        ROS_INFO("ADVANCED:");
        ROS_INFO("  1/2: Toggle RF leg on/off");
        ROS_INFO("  3/4: Toggle LF leg on/off");
        ROS_INFO("  +/-: Adjust step height");
        ROS_INFO("  [/]: Adjust step length");
        ROS_INFO("");
        ROS_INFO("Press 'h' to move to home position...");
    }
    
    void controlTimerCallback(const ros::TimerEvent& event) {
        char key = getKey();
        
        if (key != 0) {
            handleKeyInput(key);
            last_key_ = key;
        }
        
        // 歩行制御（SPACEキーが押されている間のみ）
        if (is_walking_ && last_key_ == ' ') {
            updateDualLegWalking();
        } else if (is_walking_ && last_key_ != ' ') {
            stopWalking();
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
                
            case 27:   // ESC
            case 'q':  // 緊急停止・終了
                emergencyStop();
                ros::shutdown();
                break;
                
            // 脚の個別制御
            case '1':
                legs_["RF"].active = true;
                ROS_INFO("RF leg ENABLED");
                break;
            case '2':
                legs_["RF"].active = false;
                ROS_INFO("RF leg DISABLED");
                break;
            case '3':
                legs_["LF"].active = true;
                ROS_INFO("LF leg ENABLED");
                break;
            case '4':
                legs_["LF"].active = false;
                ROS_INFO("LF leg DISABLED");
                break;
                
            // パラメータ調整
            case '+':
            case '=':
                walk_params_.step_height += 2.0;
                walk_params_.step_height = std::min(40.0, walk_params_.step_height);
                printCurrentParams();
                break;
            case '-':
                walk_params_.step_height -= 2.0;
                walk_params_.step_height = std::max(5.0, walk_params_.step_height);
                printCurrentParams();
                break;
                
            case '[':
                walk_params_.step_length -= 5.0;
                walk_params_.step_length = std::max(10.0, walk_params_.step_length);
                printCurrentParams();
                break;
            case ']':
                walk_params_.step_length += 5.0;
                walk_params_.step_length = std::min(80.0, walk_params_.step_length);
                printCurrentParams();
                break;
                
            default:
                if (is_walking_) {
                    last_key_ = 0; // 歩行停止
                }
                break;
        }
    }
    
    void printCurrentParams() {
        ROS_INFO("Params: height=%.1fmm, length=%.1fmm, cycle=%.1fs, speed=x%.1f", 
                 walk_params_.step_height, walk_params_.step_length, 
                 walk_params_.cycle_time, walk_params_.speed_multiplier);
    }
    
    void startWalking() {
        if (!is_enabled_) {
            is_enabled_ = true;
            current_time_ = 0.0;
            ROS_INFO("Dual leg walking STARTED - Hold SPACE to continue");
        }
        is_walking_ = true;
    }
    
    void stopWalking() {
        if (is_walking_) {
            is_walking_ = false;
            ROS_INFO("Dual leg walking STOPPED");
        }
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
        
        ROS_INFO("Moving both legs to HOME position");
    }
    
    void emergencyStop() {
        is_walking_ = false;
        is_enabled_ = false;
        moveToHome();
        ROS_WARN("EMERGENCY STOP - Moving to home positions");
    }
    
    void updateDualLegWalking() {
        current_time_ += 0.02 * walk_params_.speed_multiplier;
        
        double effective_cycle_time = walk_params_.cycle_time / walk_params_.speed_multiplier;
        
        for (const auto& leg_pair : legs_) {
            const std::string& leg_id = leg_pair.first;
            const LegConfig& config = leg_pair.second;
            
            if (!config.active) continue;  // 無効な脚はスキップ
            
            // 脚ごとの位相オフセット計算
            double leg_phase_offset = 0.0;
            if (leg_id == "LF") {
                leg_phase_offset = walk_params_.phase_offset;  // LFは180度遅れ
            }
            
            double phase = fmod(current_time_ + leg_phase_offset * effective_cycle_time, 
                              effective_cycle_time) / effective_cycle_time;
            
            dual_leg_controller::LegPosition cmd;
            
            if (phase < 0.5) {
                // スタンス期（接地期）
                double stance_ratio = phase / 0.5;
                cmd.x = config.home_x + walk_params_.step_length/2.0 - walk_params_.step_length * stance_ratio;
                cmd.y = config.home_y;
                cmd.z = config.home_z;
            } else {
                // スイング期（遊脚期）
                double swing_ratio = (phase - 0.5) / 0.5;
                double swing_angle = swing_ratio * M_PI;
                
                cmd.x = config.home_x - walk_params_.step_length/2.0 + walk_params_.step_length * swing_ratio;
                cmd.y = config.home_y;
                cmd.z = config.home_z + walk_params_.step_height * sin(swing_angle);
            }
            
            leg_pos_pubs_[leg_id].publish(cmd);
        }
        
        // デバッグ出力（2秒に1回）
        if (static_cast<int>(current_time_ * 50) % 100 == 0) {
            double rf_phase = fmod(current_time_, effective_cycle_time) / effective_cycle_time;
            double lf_phase = fmod(current_time_ + walk_params_.phase_offset * effective_cycle_time, 
                                 effective_cycle_time) / effective_cycle_time;
            ROS_INFO("Phases - RF: %.2f, LF: %.2f", rf_phase, lf_phase);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "safe_dual_leg_test");
    
    ROS_INFO("Starting Safe Dual Leg Test (RF-LF)...");
    ROS_WARN("Using parameters validated from single leg test");
    
    SafeDualLegTest test;
    
    ros::spin();
    return 0;
}
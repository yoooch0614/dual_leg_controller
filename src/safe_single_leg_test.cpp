// 安全なキーボード制御式単脚歩行テスト
#include <ros/ros.h>
#include "dual_leg_controller/LegCommand.h"
#include "dual_leg_controller/LegPosition.h"
#include <std_msgs/String.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class SafeSingleLegTest {
private:
    ros::NodeHandle nh_;
    ros::Publisher leg_pos_pub_;
    ros::Publisher leg_angle_pub_;
    ros::Timer control_timer_;
    
    // 安全パラメータ（動的調整可能）
    struct SafetyParams {
        double step_height_min;      // 最小ステップ高さ [mm]
        double step_height_max;      // 最大ステップ高さ [mm]
        double step_length_min;      // 最小ステップ長さ [mm]
        double step_length_max;      // 最大ステップ長さ [mm]
        double cycle_time_min;       // 最小周期 [s]
        double cycle_time_max;       // 最大周期 [s]
        double z_limit_min;          // Z軸最小値（地面より下に行かない）[mm]
        double z_limit_max;          // Z軸最大値（高く上がりすぎない）[mm]
        double x_limit_min;          // X軸最小値 [mm]
        double x_limit_max;          // X軸最大値 [mm]
        double y_limit_min;          // Y軸最小値 [mm]
        double y_limit_max;          // Y軸最大値 [mm]
    } safety_params_;
    
    // 現在の歩行パラメータ
    struct WalkParams {
        double step_height;          // 現在のステップ高さ [mm]
        double step_length;          // 現在のステップ長さ [mm]
        double cycle_time;           // 現在の歩行周期 [s]
        double speed_multiplier;     // 速度倍率（0.1～2.0）
    } walk_params_;
    
    // 脚の基準位置（安全な位置）
    struct {
        double x, y, z;
    } home_position_;
    
    // 制御状態
    bool is_walking_;
    bool is_enabled_;
    double current_time_;
    char last_key_;
    
    // キーボード入力用
    struct termios old_tio_, new_tio_;
    bool keyboard_initialized_;

public:
    SafeSingleLegTest() : nh_(""), is_walking_(false), is_enabled_(false), 
                         current_time_(0.0), last_key_(0), keyboard_initialized_(false) {
        
        // 安全パラメータの初期化（保守的な値に設定）
        nh_.param("safety/step_height_min", safety_params_.step_height_min, 5.0);
        nh_.param("safety/step_height_max", safety_params_.step_height_max, 40.0);
        nh_.param("safety/step_length_min", safety_params_.step_length_min, 10.0);
        nh_.param("safety/step_length_max", safety_params_.step_length_max, 60.0);
        nh_.param("safety/cycle_time_min", safety_params_.cycle_time_min, 1.0);
        nh_.param("safety/cycle_time_max", safety_params_.cycle_time_max, 8.0);
        nh_.param("safety/z_limit_min", safety_params_.z_limit_min, -120.0);
        nh_.param("safety/z_limit_max", safety_params_.z_limit_max, -50.0);
        nh_.param("safety/x_limit_min", safety_params_.x_limit_min, 80.0);
        nh_.param("safety/x_limit_max", safety_params_.x_limit_max, 200.0);
        nh_.param("safety/y_limit_min", safety_params_.y_limit_min, -50.0);
        nh_.param("safety/y_limit_max", safety_params_.y_limit_max, 50.0);
        
        // 歩行パラメータの初期化（安全な値）
        walk_params_.step_height = 15.0;    // 小さめから開始
        walk_params_.step_length = 25.0;    // 小さめから開始
        walk_params_.cycle_time = 3.0;      // ゆっくりから開始
        walk_params_.speed_multiplier = 0.5; // 半分の速度から開始
        
        // 脚の基準位置設定（RF脚想定、安全な位置）
        nh_.param("home_position/x", home_position_.x, 140.0);
        nh_.param("home_position/y", home_position_.y, 0.0);
        nh_.param("home_position/z", home_position_.z, -90.0);
        
        // 基準位置の安全性チェック
        if (!isPositionSafe(home_position_.x, home_position_.y, home_position_.z)) {
            ROS_ERROR("Home position is not safe! Adjusting...");
            home_position_.x = 140.0;
            home_position_.y = 0.0;
            home_position_.z = -90.0;
        }
        
        // Publisher設定
        leg_pos_pub_ = nh_.advertise<dual_leg_controller::LegPosition>(
            "/asterisk/leg/RF/command/foot_position", 1);
        leg_angle_pub_ = nh_.advertise<dual_leg_controller::LegCommand>(
            "/asterisk/leg/RF/command/joint_angles", 1);
        
        // タイマー設定（50Hz、高い頻度で安全確認）
        control_timer_ = nh_.createTimer(ros::Duration(0.02), 
            &SafeSingleLegTest::controlTimerCallback, this);
        
        // キーボード設定
        initializeKeyboard();
        
        // 最初はホームポジションに移動
        moveToHome();
        
        printInstructions();
    }
    
    ~SafeSingleLegTest() {
        if (keyboard_initialized_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
        }
        // 終了時は必ずホームポジションに戻る
        moveToHome();
        ros::Duration(0.5).sleep(); // 少し待つ
    }
    
    void initializeKeyboard() {
        // 現在の端末設定を保存
        tcgetattr(STDIN_FILENO, &old_tio_);
        new_tio_ = old_tio_;
        
        // 非正規モードに設定（文字単位での入力）
        new_tio_.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);
        
        // ノンブロッキングに設定
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
        ROS_INFO("=== SAFE SINGLE LEG WALK TEST ===");
        ROS_INFO("SAFETY FEATURES:");
        ROS_INFO("  - Only moves when key is pressed");
        ROS_INFO("  - Position limits enforced");
        ROS_INFO("  - Emergency stop available");
        ROS_INFO("");
        ROS_INFO("CONTROLS:");
        ROS_INFO("  SPACE: Start/Continue walking (hold to walk)");
        ROS_INFO("  s: Stop and hold position");
        ROS_INFO("  h: Return to home position");
        ROS_INFO("  ESC/q: Emergency stop and exit");
        ROS_INFO("");
        ROS_INFO("PARAMETER ADJUSTMENT:");
        ROS_INFO("  +/-: Increase/Decrease step height (%.1fmm)", walk_params_.step_height);
        ROS_INFO("  [/]: Increase/Decrease step length (%.1fmm)", walk_params_.step_length);
        ROS_INFO("  {/}: Increase/Decrease cycle time (%.1fs)", walk_params_.cycle_time);
        ROS_INFO("  f/s: Faster/Slower motion (x%.1f)", walk_params_.speed_multiplier);
        ROS_INFO("");
        ROS_INFO("CURRENT POSITION: [%.1f, %.1f, %.1f]", 
                 home_position_.x, home_position_.y, home_position_.z);
        ROS_INFO("Press 'h' to start at home position...");
    }
    
    void printCurrentParams() {
        ROS_INFO("Current params: height=%.1fmm, length=%.1fmm, cycle=%.1fs, speed=x%.1f", 
                 walk_params_.step_height, walk_params_.step_length, 
                 walk_params_.cycle_time, walk_params_.speed_multiplier);
    }
    
    void controlTimerCallback(const ros::TimerEvent& event) {
        char key = getKey();
        
        // キー入力処理
        if (key != 0) {
            handleKeyInput(key);
            last_key_ = key;
        }
        
        // 歩行制御（SPACEキーが押されている間のみ）
        if (is_walking_ && last_key_ == ' ') {
            updateWalkingMotion();
        } else if (is_walking_ && last_key_ != ' ') {
            // SPACEキーが離されたら歩行停止
            stopWalking();
        }
    }
    
    void handleKeyInput(char key) {
        switch (key) {
            case ' ':  // スペースキー：歩行開始/継続
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
            case 'q':  // q キー：緊急停止・終了
                emergencyStop();
                ros::shutdown();
                break;
                
            // パラメータ調整
            case '+':
            case '=':
                adjustStepHeight(2.0);
                break;
            case '-':
                adjustStepHeight(-2.0);
                break;
                
            case '[':
                adjustStepLength(-5.0);
                break;
            case ']':
                adjustStepLength(5.0);
                break;
                
            case '{':
                adjustCycleTime(0.2);
                break;
            case '}':
                adjustCycleTime(-0.2);
                break;
                
            case 'f':
                adjustSpeedMultiplier(0.1);
                break;
            case 'w':  // 's'は停止に使うので'w'を使用
                adjustSpeedMultiplier(-0.1);
                break;
                
            default:
                // 無効なキーの場合、歩行停止
                if (is_walking_) {
                    last_key_ = 0; // キーをクリアして歩行停止
                }
                break;
        }
    }
    
    void startWalking() {
        if (!is_enabled_) {
            is_enabled_ = true;
            current_time_ = 0.0;
            ROS_INFO("Walking STARTED - Hold SPACE to continue");
        }
        is_walking_ = true;
    }
    
    void stopWalking() {
        if (is_walking_) {
            is_walking_ = false;
            ROS_INFO("Walking STOPPED");
        }
    }
    
    void moveToHome() {
        is_walking_ = false;
        is_enabled_ = false;
        
        dual_leg_controller::LegPosition cmd;
        cmd.x = home_position_.x;
        cmd.y = home_position_.y;
        cmd.z = home_position_.z;
        
        leg_pos_pub_.publish(cmd);
        ROS_INFO("Moving to HOME position: [%.1f, %.1f, %.1f]", cmd.x, cmd.y, cmd.z);
    }
    
    void emergencyStop() {
        is_walking_ = false;
        is_enabled_ = false;
        moveToHome();
        ROS_WARN("EMERGENCY STOP - Moving to home position");
    }
    
    void updateWalkingMotion() {
        current_time_ += 0.02 * walk_params_.speed_multiplier;
        
        // 歩行周期の位相計算（0～1）
        double effective_cycle_time = walk_params_.cycle_time / walk_params_.speed_multiplier;
        double phase = fmod(current_time_, effective_cycle_time) / effective_cycle_time;
        
        dual_leg_controller::LegPosition cmd;
        
        if (phase < 0.5) {
            // スタンス期（接地期）: 直線的に後方移動
            double stance_ratio = phase / 0.5;
            cmd.x = home_position_.x + walk_params_.step_length/2.0 - walk_params_.step_length * stance_ratio;
            cmd.y = home_position_.y;
            cmd.z = home_position_.z;
        } else {
            // スイング期（遊脚期）: 円弧軌道で前方移動
            double swing_ratio = (phase - 0.5) / 0.5;
            double swing_angle = swing_ratio * M_PI;
            
            cmd.x = home_position_.x - walk_params_.step_length/2.0 + walk_params_.step_length * swing_ratio;
            cmd.y = home_position_.y;
            cmd.z = home_position_.z + walk_params_.step_height * sin(swing_angle);
        }
        
        // 安全性チェック
        if (isPositionSafe(cmd.x, cmd.y, cmd.z)) {
            leg_pos_pub_.publish(cmd);
        } else {
            ROS_ERROR("Unsafe position detected! [%.1f, %.1f, %.1f] - Emergency stop", 
                     cmd.x, cmd.y, cmd.z);
            emergencyStop();
        }
        
        // デバッグ出力（2秒に1回）
        if (static_cast<int>(current_time_ * 50) % 100 == 0) {
            ROS_INFO("Phase: %.2f, Position: [%.1f, %.1f, %.1f]", 
                     phase, cmd.x, cmd.y, cmd.z);
        }
    }
    
    bool isPositionSafe(double x, double y, double z) {
        return (x >= safety_params_.x_limit_min && x <= safety_params_.x_limit_max &&
                y >= safety_params_.y_limit_min && y <= safety_params_.y_limit_max &&
                z >= safety_params_.z_limit_min && z <= safety_params_.z_limit_max);
    }
    
    void adjustStepHeight(double delta) {
        walk_params_.step_height += delta;
        walk_params_.step_height = std::max(safety_params_.step_height_min, 
                                          std::min(safety_params_.step_height_max, walk_params_.step_height));
        printCurrentParams();
    }
    
    void adjustStepLength(double delta) {
        walk_params_.step_length += delta;
        walk_params_.step_length = std::max(safety_params_.step_length_min, 
                                          std::min(safety_params_.step_length_max, walk_params_.step_length));
        printCurrentParams();
    }
    
    void adjustCycleTime(double delta) {
        walk_params_.cycle_time += delta;
        walk_params_.cycle_time = std::max(safety_params_.cycle_time_min, 
                                         std::min(safety_params_.cycle_time_max, walk_params_.cycle_time));
        printCurrentParams();
    }
    
    void adjustSpeedMultiplier(double delta) {
        walk_params_.speed_multiplier += delta;
        walk_params_.speed_multiplier = std::max(0.1, std::min(2.0, walk_params_.speed_multiplier));
        printCurrentParams();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "safe_single_leg_test");
    
    ROS_INFO("Starting Safe Single Leg Test...");
    ROS_WARN("Make sure the robot is in a safe position before starting!");
    
    SafeSingleLegTest test;
    
    ros::spin();
    return 0;
}
// ステップ幅修正版2脚歩行制御
#include <ros/ros.h>
#include "dual_leg_controller/LegPosition.h"
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <map>

class FixedTwoLegWalk {
private:
    ros::NodeHandle nh_;
    std::map<std::string, ros::Publisher> leg_pos_pubs_;
    ros::Timer control_timer_;
    
    // 歩行パラメータ
    double step_height_;
    double step_length_;       // 実際のステップ長さ [mm]
    double cycle_time_;
    double robot_velocity_x_;
    double robot_velocity_y_;
    
    // 制御状態
    bool is_walking_;
    bool is_enabled_;
    double current_time_;
    
    // キーボード
    struct termios old_termios_;
    bool keyboard_initialized_;

public:
    FixedTwoLegWalk() : nh_(""), is_walking_(false), is_enabled_(false), 
                       current_time_(0.0), keyboard_initialized_(false),
                       step_height_(17.0), step_length_(60.0), cycle_time_(1.2),
                       robot_velocity_x_(20.0), robot_velocity_y_(0.0) {
        
        // トピック設定
        leg_pos_pubs_["RF"] = nh_.advertise<dual_leg_controller::LegPosition>(
            "/asterisk/leg/RF/command/foot_position", 1);
        leg_pos_pubs_["LF"] = nh_.advertise<dual_leg_controller::LegPosition>(
            "/asterisk/leg/LF/command/foot_position", 1);
        
        // タイマー
        control_timer_ = nh_.createTimer(ros::Duration(0.02), 
                                       &FixedTwoLegWalk::controlCallback, this);
        
        // キーボード初期化
        initializeKeyboard();
        
        ROS_INFO("Fixed Two Leg Walk Controller initialized");
        printInstructions();
        moveToHome();
    }
    
    ~FixedTwoLegWalk() {
        if (keyboard_initialized_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
        }
    }

private:
    void controlCallback(const ros::TimerEvent&) {
        handleKeyboardInput();
        
        if (is_walking_ && is_enabled_) {
            updateFixedWalking();
        }
    }
    
    void updateFixedWalking() {
        current_time_ += 0.02;
        
        // RF脚の処理
        processLegFixed("RF", 0.0, 0.0);  // 0度、位相オフセット0
        
        // LF脚の処理  
        processLegFixed("LF", 60.0, 0.5);  // 60度、位相オフセット0.5
    }
    
    void processLegFixed(const std::string& leg_id, double attach_angle_deg, double phase_offset) {
        // 位相計算
        double phase = fmod(current_time_ / cycle_time_ + phase_offset, 1.0);
        
        // 座標変換
        double angle_rad = attach_angle_deg * M_PI / 180.0;
        
        // ★修正★ ロボット速度ではなく、固定ステップ長さを使用
        double leg_vx = robot_velocity_x_ * cos(angle_rad) + robot_velocity_y_ * sin(angle_rad);
        double leg_vy = -robot_velocity_x_ * sin(angle_rad) + robot_velocity_y_ * cos(angle_rad);
        
        // ★重要★ ステップ幅を velocity × time ではなく、直接指定
        // 前の実装: leg_step_x = leg_vx * cycle_time_ / 1000.0;  // これが0.024mmしかなかった
        // 修正版: 実際のステップ長さを使用
        
        double step_magnitude = sqrt(leg_vx * leg_vx + leg_vy * leg_vy);
        double leg_step_x, leg_step_y;
        
        if (step_magnitude > 0.1) {  // 最小しきい値
            // 方向は保持、大きさを step_length_ に設定
            leg_step_x = (leg_vx / step_magnitude) * step_length_;
            leg_step_y = (leg_vy / step_magnitude) * step_length_;
        } else {
            // 前進のみの場合のデフォルト
            leg_step_x = step_length_ * cos(angle_rad);
            leg_step_y = step_length_ * (-sin(angle_rad));
        }
        
        ROS_INFO_THROTTLE(1.0, "%s脚: 計算されたステップ=(%.1f,%.1f)mm", 
                          leg_id.c_str(), leg_step_x, leg_step_y);
        
        // スイング/スタンス期計算
        bool is_swing_phase = (phase < 0.5);
        double step_progress, x_offset, y_offset, z_offset;
        
        if (is_swing_phase) {
            // スイング期：前方移動（空中）
            step_progress = phase * 2.0;  // 0→1
            x_offset = -leg_step_x * 0.5 + leg_step_x * step_progress;
            y_offset = -leg_step_y * 0.5 + leg_step_y * step_progress;
            double swing_height = 4.0 * step_height_ * step_progress * (1.0 - step_progress);
            z_offset = swing_height;
        } else {
            // スタンス期：後方移動（地面接触）
            step_progress = (phase - 0.5) * 2.0;  // 0→1
            x_offset = leg_step_x * 0.5 - leg_step_x * step_progress;
            y_offset = leg_step_y * 0.5 - leg_step_y * step_progress;
            z_offset = 0.0;
        }
        
        // 最終位置計算
        double home_x = 140.0, home_y = 0.0, home_z = -90.0;
        
        dual_leg_controller::LegPosition cmd;
        cmd.x = home_x + x_offset;
        cmd.y = home_y + y_offset;
        cmd.z = home_z + z_offset;
        
        // 指令送信
        leg_pos_pubs_[leg_id].publish(cmd);
        
        ROS_INFO_THROTTLE(1.0, "%s脚: 最終位置=(%.1f,%.1f,%.1f), オフセット=(%.1f,%.1f), phase=%.2f", 
                          leg_id.c_str(), cmd.x, cmd.y, cmd.z, x_offset, y_offset, phase);
    }
    
    void printInstructions() {
        ROS_INFO("=== FIXED TWO LEG WALK TEST ===");
        ROS_INFO("ステップ幅を大幅に改善");
        ROS_INFO(" ");
        ROS_INFO("CONTROLS:");
        ROS_INFO("  SPACE: Start/Stop walking");
        ROS_INFO("  +/-: Step length (現在: %.1fmm)", step_length_);
        ROS_INFO("  w/s: Step height (現在: %.1fmm)", step_height_);
        ROS_INFO("  r: Show expected step size");
        ROS_INFO("  h: Home position");
        ROS_INFO("  q: Exit");
        ROS_INFO(" ");
        showExpectedSteps();
    }
    
    void showExpectedSteps() {
        ROS_INFO("期待されるステップ幅:");
        
        // RF脚（0度）
        double rf_step_x = step_length_ * cos(0.0);
        double rf_step_y = step_length_ * (-sin(0.0));
        ROS_INFO("  RF脚: X範囲 %.1f〜%.1f (幅%.1fmm), Y=%.1f", 
                 140.0 - rf_step_x/2, 140.0 + rf_step_x/2, rf_step_x, rf_step_y);
        
        // LF脚（60度）
        double lf_step_x = step_length_ * cos(60.0 * M_PI / 180.0);
        double lf_step_y = step_length_ * (-sin(60.0 * M_PI / 180.0));
        ROS_INFO("  LF脚: X範囲 %.1f〜%.1f (幅%.1fmm), Y範囲 %.1f〜%.1f (幅%.1fmm)", 
                 140.0 - lf_step_x/2, 140.0 + lf_step_x/2, lf_step_x,
                 0.0 - lf_step_y/2, 0.0 + lf_step_y/2, fabs(lf_step_y));
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
            case '+':
                step_length_ = std::min(100.0, step_length_ + 10.0);
                ROS_INFO("Step length: %.1fmm", step_length_);
                showExpectedSteps();
                break;
            case '-':
                step_length_ = std::max(10.0, step_length_ - 10.0);
                ROS_INFO("Step length: %.1fmm", step_length_);
                showExpectedSteps();
                break;
            case 'w':
                step_height_ = std::min(30.0, step_height_ + 2.0);
                ROS_INFO("Step height: %.1fmm", step_height_);
                break;
            case 's':
                step_height_ = std::max(5.0, step_height_ - 2.0);
                ROS_INFO("Step height: %.1fmm", step_height_);
                break;
            case 'r':
                showExpectedSteps();
                break;
            case 'h':
                moveToHome();
                break;
            case 'q':
                ROS_INFO("Exiting...");
                moveToHome();
                ros::shutdown();
                break;
        }
    }
    
    void toggleWalking() {
        if (!is_enabled_) {
            is_enabled_ = true;
            ROS_INFO("Legs enabled");
        }
        
        is_walking_ = !is_walking_;
        if (is_walking_) {
            current_time_ = 0.0;
            ROS_INFO("Walking started - Step length: %.1fmm", step_length_);
            showExpectedSteps();
        } else {
            ROS_INFO("Walking stopped");
        }
    }
    
    void moveToHome() {
        is_walking_ = false;
        is_enabled_ = false;
        
        dual_leg_controller::LegPosition cmd;
        cmd.x = 140.0;
        cmd.y = 0.0;
        cmd.z = -90.0;
        
        leg_pos_pubs_["RF"].publish(cmd);
        leg_pos_pubs_["LF"].publish(cmd);
        
        ROS_INFO("Moved to home position: (%.1f, %.1f, %.1f)", cmd.x, cmd.y, cmd.z);
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
    ros::init(argc, argv, "fixed_two_leg_walk");
    
    try {
        FixedTwoLegWalk walker;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    
    return 0;
}
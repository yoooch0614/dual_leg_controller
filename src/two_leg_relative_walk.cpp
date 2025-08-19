// 簡易版2脚相対位置歩行制御（エラー修正版）
#include <ros/ros.h>
#include "dual_leg_controller/LegPosition.h"
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <map>

class SimpleTwoLegWalk {
private:
    ros::NodeHandle nh_;
    std::map<std::string, ros::Publisher> leg_pos_pubs_;
    ros::Timer control_timer_;
    
    // 脚の設定
    struct LegConfig {
        std::string leg_id;
        double attach_angle;        // 度
        double home_x, home_y, home_z;
        bool enabled;
    };
    
    std::map<std::string, LegConfig> legs_;
    
    // 歩行パラメータ
    double step_height_;
    double step_length_;
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
    SimpleTwoLegWalk() : nh_(""), is_walking_(false), is_enabled_(false), 
                        current_time_(0.0), keyboard_initialized_(false),
                        step_height_(17.0), step_length_(60.0), cycle_time_(1.2),
                        robot_velocity_x_(20.0), robot_velocity_y_(0.0) {
        
        // 脚設定
        legs_["RF"] = {"RF", 0.0, 140.0, 0.0, -90.0, true};
        legs_["LF"] = {"LF", 60.0, 140.0, 0.0, -90.0, true};
        
        // トピック設定
        for (const auto& leg_pair : legs_) {
            const std::string& leg_id = leg_pair.first;
            std::string topic = "/asterisk/leg/" + leg_id + "/command/foot_position";
            leg_pos_pubs_[leg_id] = nh_.advertise<dual_leg_controller::LegPosition>(topic, 1);
        }
        
        // タイマー
        control_timer_ = nh_.createTimer(ros::Duration(0.02), 
                                       &SimpleTwoLegWalk::controlCallback, this);
        
        // キーボード初期化
        initializeKeyboard();
        
        ROS_INFO("Simple Two Leg Walk Controller initialized");
        printInstructions();
        moveToHome();
    }
    
    ~SimpleTwoLegWalk() {
        if (keyboard_initialized_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
        }
    }

private:
    void controlCallback(const ros::TimerEvent&) {
        handleKeyboardInput();
        
        if (is_walking_ && is_enabled_) {
            updateWalking();
        }
    }
    
    void updateWalking() {
        current_time_ += 0.02;
        
        for (const auto& leg_pair : legs_) {
            const std::string& leg_id = leg_pair.first;
            const LegConfig& config = leg_pair.second;
            
            if (!config.enabled) continue;
            
            // 位相計算
            double phase_offset = (leg_id == "LF") ? 0.5 : 0.0;
            double phase = fmod(current_time_ / cycle_time_ + phase_offset, 1.0);
            
            // 座標変換
            double angle_rad = config.attach_angle * M_PI / 180.0;
            double leg_vx = robot_velocity_x_ * cos(angle_rad) + robot_velocity_y_ * sin(angle_rad);
            double leg_vy = -robot_velocity_x_ * sin(angle_rad) + robot_velocity_y_ * cos(angle_rad);
            
            // ステップ計算
            double leg_step_x = leg_vx * cycle_time_ / 1000.0;
            double leg_step_y = leg_vy * cycle_time_ / 1000.0;
            
            // 制限
            double step_magnitude = sqrt(leg_step_x * leg_step_x + leg_step_y * leg_step_y);
            if (step_magnitude > step_length_) {
                leg_step_x = leg_step_x * step_length_ / step_magnitude;
                leg_step_y = leg_step_y * step_length_ / step_magnitude;
            }
            
            // スイング/スタンス期
            bool is_swing_phase = (phase < 0.5);
            double step_progress, x_offset, y_offset, z_offset;
            
            if (is_swing_phase) {
                // スイング期
                step_progress = phase * 2.0;
                x_offset = -leg_step_x * 0.5 + leg_step_x * step_progress;
                y_offset = -leg_step_y * 0.5 + leg_step_y * step_progress;
                double swing_height = 4.0 * step_height_ * step_progress * (1.0 - step_progress);
                z_offset = swing_height;
            } else {
                // スタンス期
                step_progress = (phase - 0.5) * 2.0;
                x_offset = leg_step_x * 0.5 - leg_step_x * step_progress;
                y_offset = leg_step_y * 0.5 - leg_step_y * step_progress;
                z_offset = 0.0;
            }
            
            // 指令送信
            dual_leg_controller::LegPosition cmd;
            cmd.x = config.home_x + x_offset;
            cmd.y = config.home_y + y_offset;
            cmd.z = config.home_z + z_offset;
            
            leg_pos_pubs_[leg_id].publish(cmd);
        }
    }
    
    void printInstructions() {
        ROS_INFO("=== SIMPLE TWO LEG WALK TEST ===");
        ROS_INFO("CONTROLS:");
        ROS_INFO("  SPACE: Start/Stop walking");
        ROS_INFO("  w/s: Forward speed +/-");
        ROS_INFO("  h: Home position");
        ROS_INFO("  q: Exit");
        ROS_INFO("Current speed: %.1f mm/s", robot_velocity_x_);
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
            case 'w':
                robot_velocity_x_ = std::min(50.0, robot_velocity_x_ + 5.0);
                ROS_INFO("Forward speed: %.1f mm/s", robot_velocity_x_);
                break;
            case 's':
                robot_velocity_x_ = std::max(0.0, robot_velocity_x_ - 5.0);
                ROS_INFO("Forward speed: %.1f mm/s", robot_velocity_x_);
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
            ROS_INFO("Walking started - speed: %.1f mm/s", robot_velocity_x_);
        } else {
            ROS_INFO("Walking stopped");
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
        
        ROS_INFO("Moved to home position");
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
    ros::init(argc, argv, "simple_two_leg_walk");
    
    try {
        SimpleTwoLegWalk walker;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    
    return 0;
}
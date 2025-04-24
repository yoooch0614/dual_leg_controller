// 方向変更対応6脚歩行制御システム
#include <ros/ros.h>
#include "dual_leg_controller/LegPosition.h"
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <map>
#include <vector>

class HexapodDirectionalController {
private:
    ros::NodeHandle nh_;
    std::map<std::string, ros::Publisher> leg_pos_pubs_;
    ros::Subscriber cmd_vel_sub_;
    ros::Timer control_timer_;
    
    // 脚の設定情報
    struct LegConfig {
        std::string leg_id;
        int leg_index;              // 0-5
        double attach_angle;        // 取り付け角度 [度]
        double home_x, home_y, home_z;  // ホーム位置（脚座標系）
        bool enabled;
        int tripod_group;          // 0 or 1 (トライポッドグループ)
    };
    
    std::map<std::string, LegConfig> legs_;
    std::vector<std::string> leg_names_;
    
    // 歩行パラメータ
    struct WalkParams {
        double step_height = 17.0;       // [mm]
        double step_length = 60.0;       // [mm] 
        double cycle_time = 1.2;         // [s]
        double speed_multiplier = 1.0;
    } walk_params_;
    
    // ロボット制御状態
    struct RobotCommand {
        double velocity_x = 0.0;     // [mm/s] 前後（ロボット座標系）
        double velocity_y = 0.0;     // [mm/s] 左右（ロボット座標系）
        double angular_z = 0.0;      // [rad/s] 回転
        double walk_direction = 0.0; // [度] 歩行方向（0度=前進）
        double walk_speed = 20.0;    // [mm/s] 歩行速度
    } robot_cmd_;
    
    // 制御状態
    bool is_walking_;
    bool is_enabled_;
    double current_time_;
    
    // キーボード制御用
    struct termios old_termios_;
    bool keyboard_initialized_;

public:
    HexapodDirectionalController() : nh_(""), is_walking_(false), is_enabled_(false), 
                                    current_time_(0.0), keyboard_initialized_(false) {
        
        // 6脚の設定
        setupHexapodConfiguration();
        
        // トピック設定
        for (const auto& leg_pair : legs_) {
            const std::string& leg_id = leg_pair.first;
            std::string topic = "/asterisk/leg/" + leg_id + "/command/foot_position";
            leg_pos_pubs_[leg_id] = nh_.advertise<dual_leg_controller::LegPosition>(topic, 1);
        }
        
        // cmd_vel購読（将来の拡張用）
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, 
                                   &HexapodDirectionalController::cmdVelCallback, this);
        
        // 制御タイマー（50Hz）
        control_timer_ = nh_.createTimer(ros::Duration(0.02), 
                                       &HexapodDirectionalController::controlCallback, this);
        
        // キーボード初期化
        initializeKeyboard();
        
        ROS_INFO("Hexapod Directional Controller initialized");
        printInstructions();
        moveToHome();
    }
    
    ~HexapodDirectionalController() {
        if (keyboard_initialized_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
        }
    }

private:
    void setupHexapodConfiguration() {
        // 実際の脚配置に基づく設定（修正版）
        leg_names_ = {"RF", "LF", "LM", "LB", "RB", "RM"};
        
        // 各脚の取り付け角度（正しい順序）
        double attach_angles[6] = {0, 300, 240, 180, 120, 60};  // 度
        
        // トライポッドグループ分け
        int tripod_groups[6] = {0, 1, 0, 1, 0, 1};
        
        for (int i = 0; i < 6; i++) {
            legs_[leg_names_[i]] = {
                leg_names_[i],
                i,
                attach_angles[i],
                140.0, 0.0, -90.0,     // ホーム位置（全脚共通、脚座標系）
                true,
                tripod_groups[i]
            };
        }
        
        ROS_INFO("Hexapod Directional Configuration:");
        ROS_INFO("  Group 0: RF(0°), LM(240°), RB(120°)");
        ROS_INFO("  Group 1: LF(300°), LB(180°), RM(60°)");
    }
    
    void controlCallback(const ros::TimerEvent&) {
        handleKeyboardInput();
        
        if (is_walking_ && is_enabled_) {
            updateDirectionalWalking();
        }
    }
    
    void updateDirectionalWalking() {
        current_time_ += 0.02 * walk_params_.speed_multiplier;
        
        double effective_cycle_time = walk_params_.cycle_time / walk_params_.speed_multiplier;
        
        // 歩行方向に基づいて速度ベクトルを計算
        double walk_direction_rad = robot_cmd_.walk_direction * M_PI / 180.0;
        robot_cmd_.velocity_x = robot_cmd_.walk_speed * cos(walk_direction_rad);
        robot_cmd_.velocity_y = robot_cmd_.walk_speed * sin(walk_direction_rad);
        
        for (const auto& leg_name : leg_names_) {
            const LegConfig& config = legs_[leg_name];
            
            if (!config.enabled) continue;
            
            processLegDirectional(config, effective_cycle_time);
        }
    }
    
    void processLegDirectional(const LegConfig& config, double effective_cycle_time) {
        // トライポッド歩行の位相計算
        double phase_offset = (config.tripod_group == 0) ? 0.0 : 0.5;
        double phase = fmod(current_time_ / effective_cycle_time + phase_offset, 1.0);
        
        // ロボット座標系の移動量を脚座標系に変換
        double attach_angle_rad = config.attach_angle * M_PI / 180.0;
        
        // 座標変換：ロボット座標系 → 脚座標系
        double leg_velocity_x =  robot_cmd_.velocity_x * cos(attach_angle_rad) + 
                                robot_cmd_.velocity_y * sin(attach_angle_rad);
        double leg_velocity_y = -robot_cmd_.velocity_x * sin(attach_angle_rad) + 
                                robot_cmd_.velocity_y * cos(attach_angle_rad);
        
        // 回転成分の追加
        double body_radius = 95.0;  // mm
        double rotation_velocity_x = -robot_cmd_.angular_z * body_radius * sin(attach_angle_rad);
        double rotation_velocity_y =  robot_cmd_.angular_z * body_radius * cos(attach_angle_rad);
        
        // 合成速度
        leg_velocity_x += rotation_velocity_x;
        leg_velocity_y += rotation_velocity_y;
        
        // ステップ幅計算
        double velocity_magnitude = sqrt(leg_velocity_x * leg_velocity_x + leg_velocity_y * leg_velocity_y);
        double leg_step_x, leg_step_y;
        
        if (velocity_magnitude > 0.1) {
            // 方向は保持、大きさを step_length_ に設定
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
        dual_leg_controller::LegPosition cmd;
        cmd.x = config.home_x + x_offset;
        cmd.y = config.home_y + y_offset;
        cmd.z = config.home_z + z_offset;
        
        leg_pos_pubs_[config.leg_id].publish(cmd);
    }
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // cmd_velからロボット制御指令を受信
        robot_cmd_.velocity_x = msg->linear.x * 1000.0;   // m/s → mm/s
        robot_cmd_.velocity_y = msg->linear.y * 1000.0;   // m/s → mm/s  
        robot_cmd_.angular_z = msg->angular.z;            // rad/s
        
        // 速度ベクトルから方向と速度を計算
        double speed = sqrt(robot_cmd_.velocity_x * robot_cmd_.velocity_x + 
                           robot_cmd_.velocity_y * robot_cmd_.velocity_y);
        if (speed > 0.1) {
            robot_cmd_.walk_direction = atan2(robot_cmd_.velocity_y, robot_cmd_.velocity_x) * 180.0 / M_PI;
            robot_cmd_.walk_speed = speed;
        }
    }
    
    void printInstructions() {
        ROS_INFO("=== HEXAPOD DIRECTIONAL WALKING CONTROLLER ===");
        ROS_INFO("6脚方向変更対応歩行システム");
        ROS_INFO(" ");
        ROS_INFO("MAIN CONTROLS:");
        ROS_INFO("  SPACE: Start/Stop walking");
        ROS_INFO("  h: Home position");
        ROS_INFO("  r: Show current status");
        ROS_INFO("  ESC: Emergency stop and exit");
        ROS_INFO(" ");
        ROS_INFO("DIRECTION CONTROLS:");
        ROS_INFO("  w: Forward (0°)");
        ROS_INFO("  s: Backward (180°)");
        ROS_INFO("  a: Left (90°)");
        ROS_INFO("  d: Right (270°)");
        ROS_INFO("  z: Forward-Left (45°)");
        ROS_INFO("  c: Forward-Right (315°)");
        ROS_INFO("  x: Backward-Left (135°)");
        ROS_INFO("  v: Backward-Right (225°)");
        ROS_INFO(" ");
        ROS_INFO("FINE DIRECTION CONTROLS:");
        ROS_INFO("  i/k: Direction +/-15°");
        ROS_INFO("  j/l: Direction +/-45°");
        ROS_INFO("  u/o: Direction +/-5°");
        ROS_INFO(" ");
        ROS_INFO("SPEED & ROTATION:");
        ROS_INFO("  t/g: Speed +/-5mm/s");
        ROS_INFO("  q/e: Rotate left/right");
        ROS_INFO(" ");
        ROS_INFO("PARAMETER ADJUSTMENTS:");
        ROS_INFO("  +/-: Step height");
        ROS_INFO("  [/]: Step length");
        ROS_INFO("  {/}: Cycle time");
        ROS_INFO(" ");
        printCurrentStatus();
    }
    
    void printCurrentStatus() {
        ROS_INFO("=== CURRENT STATUS ===");
        ROS_INFO("Walk Direction: %.1f° (%s)", robot_cmd_.walk_direction, getDirectionName().c_str());
        ROS_INFO("Walk Speed: %.1f mm/s", robot_cmd_.walk_speed);
        ROS_INFO("Angular Speed: %.2f rad/s", robot_cmd_.angular_z);
        ROS_INFO("Step: height=%.1fmm, length=%.1fmm, cycle=%.1fs", 
                 walk_params_.step_height, walk_params_.step_length, walk_params_.cycle_time);
        ROS_INFO("Velocity Vector: (%.1f, %.1f) mm/s", robot_cmd_.velocity_x, robot_cmd_.velocity_y);
        ROS_INFO("=====================");
    }
    
    std::string getDirectionName() {
        double dir = fmod(robot_cmd_.walk_direction + 360.0, 360.0);
        
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
    
    void handleKeyboardInput() {
        char key = getKeyPress();
        if (key != 0) {
            handleKeyInput(key);
        }
    }
    
    void handleKeyInput(char key) {
        const double speed_step = 5.0;      // mm/s
        const double angular_step = 0.2;    // rad/s
        const double direction_step = 15.0; // 度
        
        switch (key) {
            case ' ':  // 歩行開始/停止
                toggleWalking();
                break;
                
            // === 8方向移動 ===
            case 'w':  // 前進
                robot_cmd_.walk_direction = 0.0;
                printCurrentStatus();
                break;
                
            case 's':  // 後退
                robot_cmd_.walk_direction = 180.0;
                printCurrentStatus();
                break;
                
            case 'a':  // 左移動
                robot_cmd_.walk_direction = 90.0;
                printCurrentStatus();
                break;
                
            case 'd':  // 右移動
                robot_cmd_.walk_direction = 270.0;
                printCurrentStatus();
                break;
                
            case 'z':  // 前進-左（45度）
                robot_cmd_.walk_direction = 45.0;
                printCurrentStatus();
                break;
                
            case 'c':  // 前進-右（315度）
                robot_cmd_.walk_direction = 315.0;
                printCurrentStatus();
                break;
                
            case 'x':  // 後退-左（135度）
                robot_cmd_.walk_direction = 135.0;
                printCurrentStatus();
                break;
                
            case 'v':  // 後退-右（225度）
                robot_cmd_.walk_direction = 225.0;
                printCurrentStatus();
                break;
                
            // === 細かい方向調整 ===
            case 'i':  // 方向+15度
                robot_cmd_.walk_direction = fmod(robot_cmd_.walk_direction + direction_step + 360.0, 360.0);
                printCurrentStatus();
                break;
                
            case 'k':  // 方向-15度
                robot_cmd_.walk_direction = fmod(robot_cmd_.walk_direction - direction_step + 360.0, 360.0);
                printCurrentStatus();
                break;
                
            case 'j':  // 方向+45度
                robot_cmd_.walk_direction = fmod(robot_cmd_.walk_direction + 45.0 + 360.0, 360.0);
                printCurrentStatus();
                break;
                
            case 'l':  // 方向-45度
                robot_cmd_.walk_direction = fmod(robot_cmd_.walk_direction - 45.0 + 360.0, 360.0);
                printCurrentStatus();
                break;
                
            case 'u':  // 方向+5度
                robot_cmd_.walk_direction = fmod(robot_cmd_.walk_direction + 5.0 + 360.0, 360.0);
                printCurrentStatus();
                break;
                
            case 'o':  // 方向-5度
                robot_cmd_.walk_direction = fmod(robot_cmd_.walk_direction - 5.0 + 360.0, 360.0);
                printCurrentStatus();
                break;
                
            // === 速度調整 ===
            case 't':  // 速度増加
                robot_cmd_.walk_speed = std::min(60.0, robot_cmd_.walk_speed + speed_step);
                printCurrentStatus();
                break;
                
            case 'g':  // 速度減少
                robot_cmd_.walk_speed = std::max(5.0, robot_cmd_.walk_speed - speed_step);
                printCurrentStatus();
                break;
                
            // === 回転 ===
            case 'q':  // 左回転
                robot_cmd_.angular_z = std::min(0.8, robot_cmd_.angular_z + angular_step);
                printCurrentStatus();
                break;
                
            case 'e':  // 右回転
                robot_cmd_.angular_z = std::max(-0.8, robot_cmd_.angular_z - angular_step);
                printCurrentStatus();
                break;
                
            // === システム制御 ===
            case 'h':  // ホームポジション
                moveToHome();
                break;
                
            case 'r':  // ステータス表示
                printCurrentStatus();
                showDirectionDiagram();
                break;
                
            case 'n':  // 全停止
                robot_cmd_.walk_speed = 0.0;
                robot_cmd_.angular_z = 0.0;
                printCurrentStatus();
                break;
                
            // === パラメータ調整 ===
            case '+':  // ステップ高さ増加
                walk_params_.step_height = std::min(30.0, walk_params_.step_height + 2.0);
                printCurrentStatus();
                break;
                
            case '-':  // ステップ高さ減少
                walk_params_.step_height = std::max(5.0, walk_params_.step_height - 2.0);
                printCurrentStatus();
                break;
                
            case '[':  // ステップ長さ減少
                walk_params_.step_length = std::max(20.0, walk_params_.step_length - 10.0);
                printCurrentStatus();
                break;
                
            case ']':  // ステップ長さ増加
                walk_params_.step_length = std::min(100.0, walk_params_.step_length + 10.0);
                printCurrentStatus();
                break;
                
            case '{':  // サイクル時間増加（遅く）
                walk_params_.cycle_time = std::min(3.0, walk_params_.cycle_time + 0.1);
                printCurrentStatus();
                break;
                
            case '}':  // サイクル時間減少（速く）
                walk_params_.cycle_time = std::max(0.8, walk_params_.cycle_time - 0.1);
                printCurrentStatus();
                break;
                
            case 27:   // ESC
                ROS_INFO("Emergency stop - Exiting...");
                moveToHome();
                ros::shutdown();
                break;
                
            default:
                break;
        }
    }
    
    void showDirectionDiagram() {
        ROS_INFO("=== DIRECTION DIAGRAM ===");
        ROS_INFO("      45°(z)   0°(w)   315°(c)");
        ROS_INFO("        \\        |        /");
        ROS_INFO("90°(a) ←  ●  → 270°(d)");
        ROS_INFO("        /        |        \\");
        ROS_INFO("     135°(x)  180°(s)  225°(v)");
        ROS_INFO("Current: %.1f° (%s)", robot_cmd_.walk_direction, getDirectionName().c_str());
        ROS_INFO("========================");
    }
    
    void toggleWalking() {
        if (!is_enabled_) {
            is_enabled_ = true;
            ROS_INFO("All legs enabled");
        }
        
        is_walking_ = !is_walking_;
        if (is_walking_) {
            current_time_ = 0.0;
            ROS_INFO("Hexapod walking started!");
            ROS_INFO("Direction: %.1f° (%s), Speed: %.1f mm/s", 
                     robot_cmd_.walk_direction, getDirectionName().c_str(), robot_cmd_.walk_speed);
        } else {
            ROS_INFO("Walking stopped");
        }
    }
    
    void moveToHome() {
        is_walking_ = false;
        is_enabled_ = false;
        robot_cmd_.angular_z = 0.0;
        
        for (const auto& leg_pair : legs_) {
            const std::string& leg_id = leg_pair.first;
            const LegConfig& config = leg_pair.second;
            
            dual_leg_controller::LegPosition cmd;
            cmd.x = config.home_x;
            cmd.y = config.home_y; 
            cmd.z = config.home_z;
            
            leg_pos_pubs_[leg_id].publish(cmd);
        }
        
        ROS_INFO("All legs moved to home position");
    }
    
    // キーボード制御関数
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
    ros::init(argc, argv, "hexapod_directional_controller");
    
    try {
        HexapodDirectionalController hexapod;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return 1;
    }
    
    return 0;
}
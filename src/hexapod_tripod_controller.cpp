// 6脚トライポッド歩行制御システム
#include <ros/ros.h>
#include "dual_leg_controller/LegPosition.h"
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <map>
#include <vector>

class HexapodTripodController {
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
        double velocity_x = 20.0;    // [mm/s] 前後
        double velocity_y = 0.0;     // [mm/s] 左右
        double angular_z = 0.0;      // [rad/s] 回転
    } robot_cmd_;
    
    // 制御状態
    bool is_walking_;
    bool is_enabled_;
    double current_time_;
    
    // キーボード制御用
    struct termios old_termios_;
    bool keyboard_initialized_;

public:
    HexapodTripodController() : nh_(""), is_walking_(false), is_enabled_(false), 
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
                                   &HexapodTripodController::cmdVelCallback, this);
        
        // 制御タイマー（50Hz）
        control_timer_ = nh_.createTimer(ros::Duration(0.02), 
                                       &HexapodTripodController::controlCallback, this);
        
        // キーボード初期化
        initializeKeyboard();
        
        ROS_INFO("Hexapod Tripod Controller initialized");
        printInstructions();
        moveToHome();
    }
    
    ~HexapodTripodController() {
        if (keyboard_initialized_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
        }
    }

private:
    void setupHexapodConfiguration() {
        // 実際の脚配置に基づく設定
        leg_names_ = {"RF", "LF", "LM", "LB", "RB", "RM"};
        
        // 各脚の取り付け角度（図に基づく正しい配置）
        double attach_angles[6] = {0, 300, 60, 180, 240, 120};  // 度
        
        // トライポッドグループ分け
        // グループ0: RF(0), LM(2), RB(4) - 同時動作
        // グループ1: LF(1), LB(3), RM(5) - 同時動作、グループ0と180度位相差
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
        
        ROS_INFO("Hexapod Tripod Configuration:");
        ROS_INFO("  Group 0 (同時動作): RF(0°), LM(60°), RB(240°)");
        ROS_INFO("  Group 1 (同時動作): LF(300°), LB(180°), RM(120°)");
        ROS_INFO(" ");
        for (const auto& leg_name : leg_names_) {
            const LegConfig& config = legs_[leg_name];
            ROS_INFO("  %s: angle=%.0f°, group=%d", 
                     config.leg_id.c_str(), config.attach_angle, config.tripod_group);
        }
    }
    
    void controlCallback(const ros::TimerEvent&) {
        handleKeyboardInput();
        
        if (is_walking_ && is_enabled_) {
            updateTripodWalking();
        }
    }
    
    void updateTripodWalking() {
        current_time_ += 0.02 * walk_params_.speed_multiplier;
        
        double effective_cycle_time = walk_params_.cycle_time / walk_params_.speed_multiplier;
        
        for (const auto& leg_name : leg_names_) {
            const LegConfig& config = legs_[leg_name];
            
            if (!config.enabled) continue;
            
            processLegTripod(config, effective_cycle_time);
        }
    }
    
    void processLegTripod(const LegConfig& config, double effective_cycle_time) {
        // トライポッド歩行の位相計算
        // グループ0: 位相0, グループ1: 位相0.5（180度ずれ）
        double phase_offset = (config.tripod_group == 0) ? 0.0 : 0.5;
        double phase = fmod(current_time_ / effective_cycle_time + phase_offset, 1.0);
        
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
        
        // ステップ幅計算（固定長さベース）
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
        
        // デバッグ情報（間引き表示）
        if ((int)(current_time_ * 5) % 10 == 0) {
            ROS_INFO_THROTTLE(2.0, "%s: step=(%.1f,%.1f), pos=(%.1f,%.1f,%.1f), %s期", 
                             config.leg_id.c_str(), leg_step_x, leg_step_y, 
                             cmd.x, cmd.y, cmd.z, is_swing_phase ? "スイング" : "スタンス");
        }
    }
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // cmd_velからロボット制御指令を受信
        robot_cmd_.velocity_x = msg->linear.x * 1000.0;   // m/s → mm/s
        robot_cmd_.velocity_y = msg->linear.y * 1000.0;   // m/s → mm/s  
        robot_cmd_.angular_z = msg->angular.z;            // rad/s
        
        ROS_INFO_THROTTLE(1.0, "cmd_vel received: vel=(%.1f,%.1f)mm/s, ang=%.2frad/s", 
                          robot_cmd_.velocity_x, robot_cmd_.velocity_y, robot_cmd_.angular_z);
    }
    
    void printInstructions() {
        ROS_INFO("=== HEXAPOD TRIPOD WALKING CONTROLLER ===");
        ROS_INFO("6脚トライポッド歩行システム");
        ROS_INFO(" ");
        ROS_INFO("Tripod Groups:");
        ROS_INFO("  Group 0: RF(0°), LM(60°), RB(240°)");
        ROS_INFO("  Group 1: LF(300°), LB(180°), RM(120°)");
        ROS_INFO(" ");
        ROS_INFO("CONTROLS:");
        ROS_INFO("  SPACE: Start/Stop walking");
        ROS_INFO("  w/s: Forward/Backward (%.1fmm/s)", robot_cmd_.velocity_x);
        ROS_INFO("  a/d: Left/Right strafe");
        ROS_INFO("  q/e: Rotate left/right");
        ROS_INFO("  h: Home position");
        ROS_INFO("  r: Show current parameters");
        ROS_INFO("  ESC: Emergency stop and exit");
        ROS_INFO(" ");
        ROS_INFO("ADJUSTMENTS:");
        ROS_INFO("  +/-: Step height (%.1fmm)", walk_params_.step_height);
        ROS_INFO("  [/]: Step length (%.1fmm)", walk_params_.step_length);
        ROS_INFO("  {/}: Cycle time (%.1fs)", walk_params_.cycle_time);
        ROS_INFO(" ");
        printCurrentParams();
    }
    
    void printCurrentParams() {
        ROS_INFO("Current: height=%.1fmm, length=%.1fmm, cycle=%.1fs", 
                 walk_params_.step_height, walk_params_.step_length, walk_params_.cycle_time);
        ROS_INFO("Robot command: vel=(%.1f,%.1f)mm/s, ang=%.2frad/s", 
                 robot_cmd_.velocity_x, robot_cmd_.velocity_y, robot_cmd_.angular_z);
    }
    
    void handleKeyboardInput() {
        char key = getKeyPress();
        if (key != 0) {
            handleKeyInput(key);
        }
    }
    
    void handleKeyInput(char key) {
        const double velocity_step = 10.0;  // mm/s
        const double angular_step = 0.2;    // rad/s
        
        switch (key) {
            case ' ':  // 歩行開始/停止
                toggleWalking();
                break;
                
            case 'w':  // 前進
                robot_cmd_.velocity_x = std::min(60.0, robot_cmd_.velocity_x + velocity_step);
                printCurrentParams();
                break;
                
            case 's':  // 後退
                robot_cmd_.velocity_x = std::max(-60.0, robot_cmd_.velocity_x - velocity_step);
                printCurrentParams();
                break;
                
            case 'a':  // 左移動
                robot_cmd_.velocity_y = std::min(40.0, robot_cmd_.velocity_y + velocity_step);
                printCurrentParams();
                break;
                
            case 'd':  // 右移動
                robot_cmd_.velocity_y = std::max(-40.0, robot_cmd_.velocity_y - velocity_step);
                printCurrentParams();
                break;
                
            case 'q':  // 左回転
                robot_cmd_.angular_z = std::min(0.8, robot_cmd_.angular_z + angular_step);
                printCurrentParams();
                break;
                
            case 'e':  // 右回転
                robot_cmd_.angular_z = std::max(-0.8, robot_cmd_.angular_z - angular_step);
                printCurrentParams();
                break;
                
            case 'h':  // ホームポジション
                moveToHome();
                break;
                
            case 'x':  // 停止
                robot_cmd_.velocity_x = 0.0;
                robot_cmd_.velocity_y = 0.0;
                robot_cmd_.angular_z = 0.0;
                printCurrentParams();
                break;
                
            case 'r':  // パラメータ表示
                printCurrentParams();
                showTripodStatus();
                break;
                
            case '+':  // ステップ高さ増加
                walk_params_.step_height = std::min(30.0, walk_params_.step_height + 2.0);
                printCurrentParams();
                break;
                
            case '-':  // ステップ高さ減少
                walk_params_.step_height = std::max(5.0, walk_params_.step_height - 2.0);
                printCurrentParams();
                break;
                
            case '[':  // ステップ長さ減少
                walk_params_.step_length = std::max(20.0, walk_params_.step_length - 10.0);
                printCurrentParams();
                break;
                
            case ']':  // ステップ長さ増加
                walk_params_.step_length = std::min(100.0, walk_params_.step_length + 10.0);
                printCurrentParams();
                break;
                
            case '{':  // サイクル時間増加（遅く）
                walk_params_.cycle_time = std::min(3.0, walk_params_.cycle_time + 0.1);
                printCurrentParams();
                break;
                
            case '}':  // サイクル時間減少（速く）
                walk_params_.cycle_time = std::max(0.8, walk_params_.cycle_time - 0.1);
                printCurrentParams();
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
    
    void toggleWalking() {
        if (!is_enabled_) {
            is_enabled_ = true;
            ROS_INFO("All legs enabled");
        }
        
        is_walking_ = !is_walking_;
        if (is_walking_) {
            current_time_ = 0.0;
            ROS_INFO("Hexapod walking started!");
            printCurrentParams();
            showTripodStatus();
        } else {
            ROS_INFO("Walking stopped");
        }
    }
    
    void showTripodStatus() {
        double phase = fmod(current_time_ / walk_params_.cycle_time, 1.0);
        
        ROS_INFO("=== TRIPOD STATUS ===");
        ROS_INFO("Current phase: %.2f", phase);
        
        if (phase < 0.5) {
            ROS_INFO("Group 0 (RF,LM,RB): SWING期 - 空中移動");
            ROS_INFO("Group 1 (LF,LB,RM): STANCE期 - 地面接触");
        } else {
            ROS_INFO("Group 0 (RF,LM,RB): STANCE期 - 地面接触");
            ROS_INFO("Group 1 (LF,LB,RM): SWING期 - 空中移動");
        }
        ROS_INFO("====================");
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
    ros::init(argc, argv, "hexapod_tripod_controller");
    
    try {
        HexapodTripodController hexapod;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return 1;
    }
    
    return 0;
}
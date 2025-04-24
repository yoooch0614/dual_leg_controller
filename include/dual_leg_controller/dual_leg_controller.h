#ifndef DUAL_LEG_CONTROLLER_H
#define DUAL_LEG_CONTROLLER_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>
#include "dual_leg_controller/LegCommand.h"
#include "dual_leg_controller/LegPosition.h"
#include "dual_leg_controller/dynamixel_sync_controller.h"

// 単脚の設定情報
struct LegConfig {
    std::string leg_id;
    uint8_t coxa_id;
    uint8_t femur_id;
    uint8_t tibia_id;
    
    // ゼロ点オフセット
    struct {
        int32_t coxa_offset;
        int32_t femur_offset;
        int32_t tibia_offset;
    } zero_positions;
    
    // 関節の回転方向
    struct {
        int coxa_direction;
        int femur_direction;
        int tibia_direction;
    } joint_directions;
};

// 制御パラメータ
struct ControlParams {
    uint32_t position_max;
    uint32_t position_min;
    uint32_t velocity_limit;
    double update_rate;
};

class DualLegController {
public:
    // コンストラクタ
    DualLegController(const std::string& device_path, const std::string& leg_id1, const std::string& leg_id2);
    // デストラクタ
    ~DualLegController();
    
    // 初期化
    bool initialize();
    // メインループ処理
    void run();

private:
    // ROS関連
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    // デバイス情報
    std::string device_path_;
    int baud_rate_;
    int protocol_version_;
    
    // 脚の設定
    std::vector<LegConfig> leg_configs_;
    
    // 制御パラメータ
    ControlParams control_params_;
    
    // リンク長（共通）
    double coxa_length_;
    double femur_length_;
    double tibia_length_;
    
    // Dynamixelコントローラ
    DynamixelSyncController* dxl_controller_;
    
    // ROSインターフェース（各脚ごと）
    std::map<std::string, ros::Subscriber> cmd_subs_;
    std::map<std::string, ros::Publisher> state_pubs_;
    std::map<std::string, ros::Publisher> fk_pos_pubs_;
    std::map<std::string, ros::Publisher> ik_angle_pubs_;
    std::map<std::string, ros::Subscriber> pos_cmd_subs_;
    
    // コールバック関数
    void commandCallback(const dual_leg_controller::LegCommand::ConstPtr& msg, const std::string& leg_id);
    void positionCommandCallback(const dual_leg_controller::LegPosition::ConstPtr& msg, const std::string& leg_id);
    
    // 脚の設定読み込み
    void loadLegConfig(const std::string& leg_id);
    
    // 運動学計算
    bool calculateForwardKinematics(double coxa_angle, double femur_angle, double tibia_angle,
                                  double& x, double& y, double& z);
    bool calculateInverseKinematics(double x, double y, double z,
                                  double& coxa_angle, double& femur_angle, double& tibia_angle);
    
    // ユーティリティ関数
    bool validateAngle(double angle);
    void normalizeAngle(double& angle);
    uint32_t angleToPosition(double angle, int32_t offset, int direction);
    double positionToAngle(uint32_t position, int32_t offset, int direction);
    int32_t degToPosition(double angle);
    
    // 検証系関数
    bool validateParameters();
    bool isWithinLimits(uint32_t position, uint32_t velocity);
    bool isNearSingularity(double coxa_angle, double femur_angle, double tibia_angle);
    
    // ヘルパー関数
    const LegConfig& getLegConfig(const std::string& leg_id);
};

#endif // DUAL_LEG_CONTROLLER_H
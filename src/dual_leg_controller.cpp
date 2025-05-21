#include "dual_leg_controller/dual_leg_controller.h"
#include <boost/make_shared.hpp>
#include <boost/bind.hpp>

// コンストラクタ
DualLegController::DualLegController(const std::string& device_path, const std::string& leg_id1, const std::string& leg_id2)
    : nh_(""), pnh_("~"), device_path_(device_path) {
    
    ROS_INFO("Initializing dual leg controller for legs %s and %s", leg_id1.c_str(), leg_id2.c_str());
    
    // Dynamixelの基本設定を読み込み
    nh_.param<int>("/dynamixel/baud_rate", baud_rate_, 57600);
    nh_.param<int>("/dynamixel/protocol_version", protocol_version_, 2);
    
    // 脚の設定を読み込む
    loadLegConfig(leg_id1);
    loadLegConfig(leg_id2);
    
    // 制御パラメータの読み込み
    int position_max_temp, position_min_temp, velocity_limit_temp;
    nh_.param("/control/position_limit/max", position_max_temp, 4095);
    nh_.param("/control/position_limit/min", position_min_temp, 0);
    nh_.param("/control/velocity_limit", velocity_limit_temp, 1023);
    control_params_.position_max = static_cast<uint32_t>(position_max_temp);
    control_params_.position_min = static_cast<uint32_t>(position_min_temp);
    control_params_.velocity_limit = static_cast<uint32_t>(velocity_limit_temp);
    
    nh_.param<double>("/control/update_rate", control_params_.update_rate, 50.0);
    
    // リンクパラメータの読み込み
    nh_.param<double>("/leg_geometry/coxa_length", coxa_length_, 25.0);
    nh_.param<double>("/leg_geometry/femur_length", femur_length_, 105.0);
    nh_.param<double>("/leg_geometry/tibia_length", tibia_length_, 105.0);
    
    // パラメータの検証
    if (!validateParameters()) {
        ROS_ERROR("Parameter validation failed");
        return;
    }
    
    // Dynamixelコントローラーの初期化
    dxl_controller_ = new DynamixelSyncController(device_path_, baud_rate_);
    
    // 各脚のROSインターフェースを設定
    for (const auto& leg_config : leg_configs_) {
        const std::string& leg_id = leg_config.leg_id;
        std::string topic_prefix = "/asterisk/leg/" + leg_id;
        
        // Subscriberとbind関数を使って各脚のコールバック関数を設定
        cmd_subs_[leg_id] = nh_.subscribe<dual_leg_controller::LegCommand>(
            topic_prefix + "/command/joint_angles", 1,
            boost::bind(&DualLegController::commandCallback, this, _1, leg_id));
        
        pos_cmd_subs_[leg_id] = nh_.subscribe<dual_leg_controller::LegPosition>(
            topic_prefix + "/command/foot_position", 1,
            boost::bind(&DualLegController::positionCommandCallback, this, _1, leg_id));
        
        // Publisher
        state_pubs_[leg_id] = nh_.advertise<dual_leg_controller::LegCommand>(
            topic_prefix + "/state/read_angle", 1);
        
        fk_pos_pubs_[leg_id] = nh_.advertise<dual_leg_controller::LegPosition>(
            topic_prefix + "/state/foot_position", 1);
        
        ik_angle_pubs_[leg_id] = nh_.advertise<dual_leg_controller::LegCommand>(
            topic_prefix + "/state/joint_angles", 1);
        
        ROS_INFO("[%s] Interface set up", leg_id.c_str());
    }
    
    ROS_INFO("DualLegController initialized successfully");
}

DualLegController::~DualLegController() {
    if (dxl_controller_) {
        delete dxl_controller_;
    }
}

// 脚の設定を読み込む
void DualLegController::loadLegConfig(const std::string& leg_id) {
    LegConfig config;
    config.leg_id = leg_id;
    
    // モーターIDの読み込み
    std::vector<int> motor_ids;
    std::string motor_param = "/dynamixel/motor_ids/" + leg_id;
    if (!nh_.getParam(motor_param, motor_ids)) {
        ROS_ERROR("[%s] Failed to get motor IDs from parameter server", leg_id.c_str());
        return;
    }
    if (motor_ids.size() != 3) {
        ROS_ERROR("[%s] Expected 3 motor IDs, got %zu", leg_id.c_str(), motor_ids.size());
        return;
    }
    config.coxa_id = motor_ids[0];
    config.femur_id = motor_ids[1];
    config.tibia_id = motor_ids[2];
    
    ROS_INFO("[%s] Using motor IDs: coxa=%d, femur=%d, tibia=%d", 
             leg_id.c_str(), config.coxa_id, config.femur_id, config.tibia_id);
    
    // ゼロ点オフセットの読み込み
    double coxa_offset_temp, femur_offset_temp, tibia_offset_temp;
    nh_.param("/joint_zero_position/coxa_offset", coxa_offset_temp, 0.0);
    nh_.param("/joint_zero_position/femur_offset", femur_offset_temp, 0.0);
    nh_.param("/joint_zero_position/tibia_offset", tibia_offset_temp, 0.0);
    
    config.zero_positions.coxa_offset = degToPosition(coxa_offset_temp);
    config.zero_positions.femur_offset = degToPosition(femur_offset_temp);
    config.zero_positions.tibia_offset = degToPosition(tibia_offset_temp);
    
    // 回転方向の読み込み
    int coxa_dir_temp, femur_dir_temp, tibia_dir_temp;
    nh_.param("/joint_direction/coxa_direction", coxa_dir_temp, 1);
    nh_.param("/joint_direction/femur_direction", femur_dir_temp, 1);
    nh_.param("/joint_direction/tibia_direction", tibia_dir_temp, 1);
    
    config.joint_directions.coxa_direction = coxa_dir_temp;
    config.joint_directions.femur_direction = femur_dir_temp;
    config.joint_directions.tibia_direction = tibia_dir_temp;
    
    // 脚の設定を追加
    leg_configs_.push_back(config);
}

// 指定した脚IDの設定を取得
const LegConfig& DualLegController::getLegConfig(const std::string& leg_id) {
    for (const auto& config : leg_configs_) {
        if (config.leg_id == leg_id) {
            return config;
        }
    }
    // 見つからない場合は最初の要素を返す（エラー処理は別途必要）
    ROS_ERROR("Leg config not found for leg_id: %s", leg_id.c_str());
    return leg_configs_[0];
}

// initialize()による初期化処理
bool DualLegController::initialize() {
    if (!dxl_controller_->initialize()) {
        ROS_ERROR("Failed to initialize Dynamixel controller");
        return false;
    }
    
    // 全てのモーターのIDを集める
    std::vector<uint8_t> all_ids;
    for (const auto& config : leg_configs_) {
        all_ids.push_back(config.coxa_id);
        all_ids.push_back(config.femur_id);
        all_ids.push_back(config.tibia_id);
    }
    
    // トルクの有効化
    if (!dxl_controller_->enableTorques(all_ids, true)) {
        ROS_ERROR("Failed to enable torques");
        return false;
    }
    
    ROS_INFO("Initialization completed successfully");
    return true;
}

bool DualLegController::isWithinLimits(uint32_t position, uint32_t velocity) {
    if (position < control_params_.position_min || 
        position > control_params_.position_max) {
        ROS_ERROR("Position %d out of limits [%d, %d]", 
                 position, control_params_.position_min, control_params_.position_max);
        return false;
    }
    
    if (velocity > control_params_.velocity_limit) {
        ROS_ERROR("Velocity %d exceeds limit %d", 
                 velocity, control_params_.velocity_limit);
        return false;
    }
    
    return true;
}

void DualLegController::run() {
    ros::Rate rate(control_params_.update_rate);
    
    while (ros::ok()) {
        // 各脚ごとに現在位置の読み取りと公開
        for (const auto& config : leg_configs_) {
            const std::string& leg_id = config.leg_id;
            std::vector<uint8_t> ids = {config.coxa_id, config.femur_id, config.tibia_id};
            std::vector<uint32_t> positions;
            
            if (dxl_controller_->syncReadPositions(ids, positions)) {
                dual_leg_controller::LegCommand state_msg;
                state_msg.coxa_angle = positionToAngle(positions[0], config.zero_positions.coxa_offset,
                                                    config.joint_directions.coxa_direction);
                state_msg.femur_angle = positionToAngle(positions[1], config.zero_positions.femur_offset,
                                                     config.joint_directions.femur_direction);
                state_msg.tibia_angle = positionToAngle(positions[2], config.zero_positions.tibia_offset,
                                                     config.joint_directions.tibia_direction);
                state_pubs_[leg_id].publish(state_msg);
            }
        }
        
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    // 引数のチェック（脚IDの取得）
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <leg_id1> <leg_id2>" << std::endl;
        return 1;
    }
    
    std::string leg_id1 = argv[1];
    std::string leg_id2 = argv[2];
    std::string device_path;
    
    // ROSの初期化（ノード名にはデバイス名を含める）
    ros::init(argc, argv, "dual_leg_controller");
    ros::NodeHandle nh;
    
    // YAML設定から対応するデバイスパスを取得
    std::string device_param = "/dynamixel/devices/" + leg_id1;
    if (!nh.getParam(device_param, device_path)) {
        ROS_ERROR("Failed to get device path for leg %s from parameter server", leg_id1.c_str());
        return 1;
    }
    
    ROS_INFO("Using device path: %s for legs %s and %s", 
            device_path.c_str(), leg_id1.c_str(), leg_id2.c_str());
    
    // 脚コントローラーの作成
    DualLegController controller(device_path, leg_id1, leg_id2);
    if (!controller.initialize()) {
        ROS_ERROR("Failed to initialize dual leg controller for %s, %s on device %s", 
                 leg_id1.c_str(), leg_id2.c_str(), device_path.c_str());
        return 1;
    }
    
    ROS_INFO("Dual leg controller started for %s, %s on device %s", 
            leg_id1.c_str(), leg_id2.c_str(), device_path.c_str());
    
    // メインループの実行
    controller.run();
    return 0;
}

// 角度の範囲が(-π~+π)なのか確認用関数
bool DualLegController::validateAngle(double angle) {
    return angle >= -M_PI && angle <= M_PI;
}

// 角度の範囲が-π~+πの範囲外である場合、範囲を内に角度の値を整える関数
void DualLegController::normalizeAngle(double& angle) {
    if (!validateAngle(angle)) {
        angle = fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0) angle += 2.0 * M_PI;
        angle -= M_PI;
        ROS_INFO("Angle converted to %f", angle);
    }
}

// angleをdynamixelが対応している角度値に変更
uint32_t DualLegController::angleToPosition(double angle, int32_t offset, int direction) {
    // 角度を-πからπの範囲に正規化
    normalizeAngle(angle);
    // 回転方向を考慮して角度を変換
    angle *= direction;
    // 角度を位置値に変換してオフセットを適用
    int32_t cal_position = static_cast<int32_t>((angle + M_PI) * 2048.0 / M_PI) + offset;
    // 0-4095の範囲に正規化
    uint32_t position = static_cast<int32_t>(((cal_position % 4096) + 4096) % 4096);
    return position;
}

double DualLegController::positionToAngle(uint32_t position, int32_t offset, int direction) {
    // オフセットを考慮した位置値を0-4095の範囲に正規化
    int32_t normalized_position = static_cast<int32_t>(position - offset);
    
    // 4096で割った余りを計算（負の値も適切に処理）
    normalized_position = ((normalized_position % 4096) + 4096) % 4096;
    
    // 正規化された位置値を角度に変換（-πからπの範囲）
    double angle = (static_cast<double>(normalized_position) * 2.0 * M_PI / 4096.0) - M_PI;
    
    // 回転方向を考慮
    angle *= direction;
    return angle;
}

int32_t DualLegController::degToPosition(double angle) {
    int32_t position = static_cast<int32_t>((angle) * 2048.0 / 180.0);
    ROS_INFO("degToPosition() : Angle Converted %f to %d", angle, position);
    return position;
}

// コマンドコールバック（FK）
void DualLegController::commandCallback(
    const dual_leg_controller::LegCommand::ConstPtr& msg, const std::string& leg_id) {
    
    // 対象の脚の設定を取得
    const LegConfig& config = getLegConfig(leg_id);
    
    std::vector<uint8_t> ids = {config.coxa_id, config.femur_id, config.tibia_id};
    std::vector<uint32_t> positions;
    std::vector<uint32_t> velocities;
    
    // 角度値の正規化
    double coxa_angle = msg->coxa_angle;
    double femur_angle = msg->femur_angle;
    double tibia_angle = msg->tibia_angle;
    
    normalizeAngle(coxa_angle);
    normalizeAngle(femur_angle);
    normalizeAngle(tibia_angle);
    
    // 正規化された角度を位置値に変換（オフセットと方向を考慮）
    uint32_t coxa_pos = angleToPosition(coxa_angle, config.zero_positions.coxa_offset, 
                                      config.joint_directions.coxa_direction);
    uint32_t femur_pos = angleToPosition(femur_angle, config.zero_positions.femur_offset,
                                       config.joint_directions.femur_direction);
    uint32_t tibia_pos = angleToPosition(tibia_angle, config.zero_positions.tibia_offset,
                                       config.joint_directions.tibia_direction);
    
    // 位置値の範囲チェック
    if (!isWithinLimits(coxa_pos, msg->velocity) ||
        !isWithinLimits(femur_pos, msg->velocity) ||
        !isWithinLimits(tibia_pos, msg->velocity)) {
        ROS_ERROR("[%s] Command values out of limits", leg_id.c_str());
        return;
    }
    
    positions = {coxa_pos, femur_pos, tibia_pos};
    
    // 速度値の設定（0-1の値を0-1023に変換）
    uint32_t velocity = static_cast<uint32_t>(msg->velocity * control_params_.velocity_limit);
    velocities = {velocity, velocity, velocity};
    
    // 同期制御の実行
    if (!dxl_controller_->syncWritePositionVelocity(ids, positions, velocities)) {
        ROS_ERROR("[%s] Failed to execute command", leg_id.c_str());
        return;
    }
    
    // 順運動学計算と位置のパブリッシュ
    double x, y, z;
    if (calculateForwardKinematics(coxa_angle, femur_angle, tibia_angle, x, y, z)) {
        dual_leg_controller::LegPosition pos_msg;
        pos_msg.x = x;
        pos_msg.y = y;
        pos_msg.z = z;
        fk_pos_pubs_[leg_id].publish(pos_msg);
    }
}

// 位置コマンドコールバック（IK）
void DualLegController::positionCommandCallback(
    const dual_leg_controller::LegPosition::ConstPtr& msg, const std::string& leg_id) {
    
    double coxa_angle, femur_angle, tibia_angle;
    
    // 逆運動学計算
    if (calculateInverseKinematics(msg->x, msg->y, msg->z,
                                 coxa_angle, femur_angle, tibia_angle)) {
        // 角度の正規化
        normalizeAngle(coxa_angle);
        normalizeAngle(femur_angle);
        normalizeAngle(tibia_angle);
        
        // 関節角度のパブリッシュ
        dual_leg_controller::LegCommand angle_msg;
        angle_msg.coxa_angle = coxa_angle;
        angle_msg.femur_angle = femur_angle;
        angle_msg.tibia_angle = tibia_angle;
        angle_msg.velocity = 0.5;  // デフォルト速度を設定
        ik_angle_pubs_[leg_id].publish(angle_msg);
        
        // 実際のモーター制御用にcommandCallbackを利用
        auto cmd_ptr = boost::make_shared<dual_leg_controller::LegCommand>(angle_msg);
        commandCallback(cmd_ptr, leg_id);
    } else {
        ROS_ERROR("[%s] Inverse kinematics calculation failed", leg_id.c_str());
    }
}

// FK計算
bool DualLegController::calculateForwardKinematics(
    double coxa_angle, double femur_angle, double tibia_angle,
    double& x, double& y, double& z) {
    
    // 股関節位置での座標計算
    x = coxa_length_ * cos(coxa_angle);
    y = coxa_length_ * sin(coxa_angle);
    
    // 大腿部の寄与
    double femur_x = femur_length_ * cos(femur_angle) * cos(coxa_angle);
    double femur_y = femur_length_ * cos(femur_angle) * sin(coxa_angle);
    double femur_z = femur_length_ * sin(femur_angle);
    
    // 脛部の寄与
    double tibia_x = tibia_length_ * cos(femur_angle + tibia_angle) * cos(coxa_angle);
    double tibia_y = tibia_length_ * cos(femur_angle + tibia_angle) * sin(coxa_angle);
    double tibia_z = tibia_length_ * sin(femur_angle + tibia_angle);
    
    // 最終位置の計算
    x += femur_x + tibia_x;
    y += femur_y + tibia_y;
    z = femur_z + tibia_z;
    
    return true;
}

// IK計算
bool DualLegController::calculateInverseKinematics(
    double x, double y, double z,
    double& coxa_angle, double& femur_angle, double& tibia_angle) {
    
    const double EPSILON = 1e-6;  // 数値計算の誤差許容範囲
    
    // Step 1: coxa角度の計算
    coxa_angle = atan2(y, x);
    
    ROS_INFO("coxa_angle = atan2(y, x) = atan2(%f, %f)", y, x);
    ROS_INFO("coxa_angle = %f rad", coxa_angle);
    
    // Step 2: coxaジョイントからの水平距離を計算
    double L = sqrt(x*x + y*y) - coxa_length_;
    
    ROS_INFO("L = sqrt(x*x + y*y) - coxa_length_ = %f", L);
    
    // Step 3: femur-tibia平面での2リンク問題を解く
    double L2 = sqrt(L*L + z*z);  // 目標点までの距離
    
    ROS_INFO("L2 = sqrt(L*L + z*z) = %f", L2);
    
    // 到達可能性チェック
    if (L2 > (femur_length_ + tibia_length_ - EPSILON)) {
        ROS_ERROR("Target position out of reach (too far): %f > %f", 
                  L2, femur_length_ + tibia_length_);
        return false;
    }
    
    if (L2 < fabs(femur_length_ - tibia_length_) + EPSILON) {
        ROS_ERROR("Target position out of reach (too close): %f < %f", 
                  L2, fabs(femur_length_ - tibia_length_));
        return false;
    }
    
    // Step 4: 余弦定理を使用してtibia角度を計算
    double cos_tibia = (L2*L2 - femur_length_*femur_length_ 
                       - tibia_length_*tibia_length_) /
                      (2.0 * femur_length_ * tibia_length_);
    
    ROS_INFO("cos_tibia = %f", cos_tibia);
    
    // 数値誤差の処理
    if (cos_tibia > 1.0 - EPSILON) {
        tibia_angle = 0.0;
    } else if (cos_tibia < -1.0 + EPSILON) {
        tibia_angle = M_PI;
    } else {
        tibia_angle = acos(cos_tibia);
    }
    
    ROS_INFO("tibia_angle = %f rad", tibia_angle);
    
    // elbow-down設定の場合は符号を反転
    tibia_angle = -tibia_angle;
    
    ROS_INFO("tibia_angle (after elbow-down correction) = %f rad", tibia_angle);
    
    // Step 5: femur角度を計算
    double gamma = atan2(z, L);  // 目標点の仰角
    double alpha = atan2(tibia_length_ * sin(tibia_angle),
                        femur_length_ + tibia_length_ * cos(tibia_angle));
    femur_angle = gamma - alpha;
    
    ROS_INFO("gamma = atan2(z, L) = %f rad", gamma);
    ROS_INFO("alpha = %f rad", alpha);
    ROS_INFO("femur_angle = %f rad", femur_angle);
    
    // Step 6: 関節角度の範囲チェック
    if (!validateAngle(coxa_angle) || 
        !validateAngle(femur_angle) || 
        !validateAngle(tibia_angle)) {
        ROS_ERROR("Joint angles out of valid range");
        return false;
    }
    
    // Step 7: 特異点チェック
    if (isNearSingularity(coxa_angle, femur_angle, tibia_angle)) {
        ROS_WARN("Near singularity detected");
        // 特異点での特別な処理が必要な場合はここで実装
    }
    
    return true;
}

// 特異点チェック
bool DualLegController::isNearSingularity(
    double coxa_angle, double femur_angle, double tibia_angle) {
    const double SINGULARITY_THRESHOLD = 0.01;  // ラジアン
    
    // 完全伸展位置のチェック
    if (fabs(tibia_angle) < SINGULARITY_THRESHOLD ||
        fabs(tibia_angle - M_PI) < SINGULARITY_THRESHOLD) {
        return true;
    }
    
    return false;
}

// パラメータ検証
bool DualLegController::validateParameters() {
    // 位置制限値のチェック
    if (control_params_.position_min >= control_params_.position_max) {
        ROS_ERROR("Invalid position limits: min >= max");
        return false;
    }
    
    // 速度制限値のチェック
    if (control_params_.velocity_limit > 1023) {
        ROS_ERROR("Invalid velocity limit: must be <= 1023");
        return false;
    }
    
    // リンク長のチェック
    if (coxa_length_ <= 0 || femur_length_ <= 0 || tibia_length_ <= 0) {
        ROS_ERROR("Invalid link lengths: must be positive");
        return false;
    }
    
    // 脚の設定チェック
    for (const auto& config : leg_configs_) {
        // 方向パラメータの検証
        if (abs(config.joint_directions.coxa_direction) != 1 ||
            abs(config.joint_directions.femur_direction) != 1 ||
            abs(config.joint_directions.tibia_direction) != 1) {
            ROS_ERROR("[%s] Invalid joint direction values: must be 1 or -1", 
                     config.leg_id.c_str());
            return false;
        }
    }
    
    return true;
}
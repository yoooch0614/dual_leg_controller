#ifndef DYNAMIXEL_SYNC_CONTROLLER_H
#define DYNAMIXEL_SYNC_CONTROLLER_H

#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <string>
#include <vector>

// Dynamixelのアドレス定義
#define ADDR_TORQUE_ENABLE          64
#define ADDR_GOAL_POSITION          116
#define ADDR_GOAL_VELOCITY          104
#define ADDR_PRESENT_POSITION       132

// データ長定義
#define LEN_GOAL_POSITION           4
#define LEN_GOAL_VELOCITY           4
#define LEN_PRESENT_POSITION        4

class DynamixelSyncController {
public:
    // コンストラクタ・デストラクタ
    DynamixelSyncController(const std::string &port_name, int baud_rate);
    ~DynamixelSyncController();
    
    // 初期化・終了
    bool initialize();
    void close();
    
    // トルク制御
    bool enableTorque(uint8_t id, bool enable);
    bool enableTorques(const std::vector<uint8_t>& ids, bool enable);
    
    // 同期制御関数
    bool syncWritePositionVelocity(
        const std::vector<uint8_t>& ids,
        const std::vector<uint32_t>& positions,
        const std::vector<uint32_t>& velocities);
    bool syncReadPositions(
        const std::vector<uint8_t>& ids,
        std::vector<uint32_t>& positions);
    
    // 個別制御関数
    bool writePosition(uint8_t id, uint32_t position);
    bool writeVelocity(uint8_t id, uint32_t velocity);
    bool readPosition(uint8_t id, uint32_t& position);
    
private:
    // Dynamixel SDK関連オブジェクト
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    dynamixel::GroupSyncWrite *syncWritePosition;
    dynamixel::GroupSyncWrite *syncWriteVelocity;
    dynamixel::GroupSyncRead *syncReadPosition;
    
    // 設定情報
    int baudRate;
    uint8_t dxl_error;
};

#endif // DYNAMIXEL_SYNC_CONTROLLER_H
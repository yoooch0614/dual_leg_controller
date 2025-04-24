#include "dual_leg_controller/dynamixel_sync_controller.h"

DynamixelSyncController::DynamixelSyncController(const std::string &port_name, int baud_rate)
    : baudRate(baud_rate), dxl_error(0) {
    // ポートハンドラとパケットハンドラの初期化
    portHandler = dynamixel::PortHandler::getPortHandler(port_name.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

    // GroupSyncWrite/Readオブジェクトの作成
    syncWritePosition = new dynamixel::GroupSyncWrite(
        portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    syncWriteVelocity = new dynamixel::GroupSyncWrite(
        portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);
    syncReadPosition = new dynamixel::GroupSyncRead(
        portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
}

DynamixelSyncController::~DynamixelSyncController() {
    close();
    delete syncWritePosition;
    delete syncWriteVelocity;
    delete syncReadPosition;
}

bool DynamixelSyncController::initialize() {
    // ポートを開く
    if (!portHandler->openPort()) {
        ROS_ERROR("Failed to open port");
        return false;
    }

    // ボーレートを設定
    if (!portHandler->setBaudRate(baudRate)) {
        ROS_ERROR("Failed to set baud rate");
        return false;
    }

    return true;
}

void DynamixelSyncController::close() {
    portHandler->closePort();
}

bool DynamixelSyncController::enableTorque(uint8_t id, bool enable) {
    int dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, id, ADDR_TORQUE_ENABLE, enable ? 1 : 0, &dxl_error);
    
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to enable torque for ID %d: %s", 
                  id, packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    if (dxl_error != 0) {
        ROS_ERROR("Error occurred while enabling torque for ID %d: %s", 
                  id, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelSyncController::enableTorques(const std::vector<uint8_t>& ids, bool enable) {
    bool success = true;
    for (auto id : ids) {
        if (!enableTorque(id, enable)) {
            success = false;
            ROS_ERROR("Failed to enable torque for ID %d", id);
        }
    }
    return success;
}

bool DynamixelSyncController::syncWritePositionVelocity(
    const std::vector<uint8_t>& ids,
    const std::vector<uint32_t>& positions,
    const std::vector<uint32_t>& velocities) {
    
    if (ids.size() != positions.size() || ids.size() != velocities.size()) {
        ROS_ERROR("ID and parameter size mismatch");
        return false;
    }

    // パラメータのクリア
    syncWritePosition->clearParam();
    syncWriteVelocity->clearParam();

    // 各モーターのパラメータを設定
    for (size_t i = 0; i < ids.size(); i++) {
        // 位置パラメータの設定
        uint8_t position_param[4];
        position_param[0] = DXL_LOBYTE(DXL_LOWORD(positions[i]));
        position_param[1] = DXL_HIBYTE(DXL_LOWORD(positions[i]));
        position_param[2] = DXL_LOBYTE(DXL_HIWORD(positions[i]));
        position_param[3] = DXL_HIBYTE(DXL_HIWORD(positions[i]));
        
        if (!syncWritePosition->addParam(ids[i], position_param)) {
            ROS_ERROR("Failed to add position parameter for ID %d", ids[i]);
            return false;
        }

        // 速度パラメータの設定
        uint8_t velocity_param[4];
        velocity_param[0] = DXL_LOBYTE(DXL_LOWORD(velocities[i]));
        velocity_param[1] = DXL_HIBYTE(DXL_LOWORD(velocities[i]));
        velocity_param[2] = DXL_LOBYTE(DXL_HIWORD(velocities[i]));
        velocity_param[3] = DXL_HIBYTE(DXL_HIWORD(velocities[i]));
        
        if (!syncWriteVelocity->addParam(ids[i], velocity_param)) {
            ROS_ERROR("Failed to add velocity parameter for ID %d", ids[i]);
            return false;
        }
    }

    // 同期書き込みの実行
    int dxl_comm_result = syncWriteVelocity->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to sync write velocities: %s", 
                  packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }

    dxl_comm_result = syncWritePosition->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to sync write positions: %s", 
                  packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }

    return true;
}

bool DynamixelSyncController::syncReadPositions(
    const std::vector<uint8_t>& ids,
    std::vector<uint32_t>& positions) {
    
    syncReadPosition->clearParam();
    positions.clear();

    // 読み取り対象のモーターを登録
    for (auto id : ids) {
        if (!syncReadPosition->addParam(id)) {
            ROS_ERROR("Failed to add parameter for reading ID %d", id);
            return false;
        }
    }

    // 同期読み取りの実行
    int dxl_comm_result = syncReadPosition->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to sync read positions: %s", 
                  packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }

    // 位置データの取得
    for (auto id : ids) {
        if (syncReadPosition->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
            uint32_t position = syncReadPosition->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            positions.push_back(position);
        } else {
            ROS_ERROR("Failed to get position data from ID %d", id);
            return false;
        }
    }

    return true;
}

bool DynamixelSyncController::writePosition(uint8_t id, uint32_t position) {
    int dxl_comm_result = packetHandler->write4ByteTxRx(
        portHandler, id, ADDR_GOAL_POSITION, position, &dxl_error);
    
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to write position for ID %d: %s", 
                  id, packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    if (dxl_error != 0) {
        ROS_ERROR("Error occurred while writing position for ID %d: %s", 
                  id, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelSyncController::writeVelocity(uint8_t id, uint32_t velocity) {
    int dxl_comm_result = packetHandler->write4ByteTxRx(
        portHandler, id, ADDR_GOAL_VELOCITY, velocity, &dxl_error);
    
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to write velocity for ID %d: %s", 
                  id, packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    if (dxl_error != 0) {
        ROS_ERROR("Error occurred while writing velocity for ID %d: %s", 
                  id, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelSyncController::readPosition(uint8_t id, uint32_t& position) {
    int dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler, id, ADDR_PRESENT_POSITION, &position, &dxl_error);
    
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to read position from ID %d: %s", 
                  id, packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    if (dxl_error != 0) {
        ROS_ERROR("Error occurred while reading position from ID %d: %s", 
                  id, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}
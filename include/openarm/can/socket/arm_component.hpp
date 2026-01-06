#pragma once

#include <vector>

#include "../../canbus/can_socket.hpp"
#include "../../damiao_motor/dm_motor.hpp"
#include "../../damiao_motor/dm_motor_device_collection.hpp"

class ArmComponent : public DMDeviceCollection {
public:
    ArmComponent(CANSocket& can_socket);
    ~ArmComponent() = default;

    void init_motor_devices(const std::vector<MotorType>& motor_types,
                            const std::vector<uint32_t>& send_can_ids,
                            const std::vector<uint32_t>& recv_can_ids);

private:
    std::vector<Motor> motors_;
};

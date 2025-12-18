#pragma once

#include <vector>

#include "../../canbus/can_socket.hpp"
#include "../../damiao_motor/dm_motor.hpp"
#include "../../damiao_motor/dm_motor_device_collection.hpp"

namespace openarm::can::socket {

class ArmComponent : public damiao_motor::DMDeviceCollection {
public:
    ArmComponent(canbus::CANSocket& can_socket);
    ~ArmComponent() = default;

    void init_motor_devices(const std::vector<damiao_motor::MotorType>& motor_types,
                            const std::vector<uint32_t>& send_can_ids,
                            const std::vector<uint32_t>& recv_can_ids);

private:
    std::vector<damiao_motor::Motor> motors_;
};

}  // namespace openarm::can::socket

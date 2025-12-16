#pragma once

#include "../canbus/common.h"
#include "../canbus/can_device.hpp"
#include "../canbus/can_socket_ex.hpp"
#include "dm_motor.hpp"
#include "dm_motor_control.hpp"

namespace openarm::damiao_motor {
enum CallbackMode {
    STATE,
    PARAM,
    // discard
    IGNORE
};

class DMCANDevice : public canbus::CANDevice {
public:
    explicit DMCANDevice(Motor& motor, canid_t recv_can_mask);
    void callback(const can_frame& frame);

    // Create frame from data array
    can_frame_ex create_can_frame(canid_t send_can_id, std::vector<uint8_t> data);
    // Getter method to access motor state
    Motor& get_motor() { return motor_; }
    void set_callback_mode(CallbackMode callback_mode) { callback_mode_ = callback_mode; }

private:
    std::vector<uint8_t> get_data_from_frame(const can_frame& frame);
    Motor& motor_;
    CallbackMode callback_mode_;
};
}  // namespace openarm::damiao_motor

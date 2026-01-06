#include <openarm/damiao_motor/dm_motor.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>
#include <openarm/damiao_motor/dm_motor_device.hpp>

DMCANDevice::DMCANDevice(Motor& motor, canid_t recv_can_mask)
    : CANDevice(motor.get_send_can_id(), motor.get_recv_can_id(), recv_can_mask),
      motor_(motor),
      callback_mode_(CallbackMode::STATE) {}

std::vector<uint8_t> DMCANDevice::get_data_from_frame(const can_frame& frame) {
    return std::vector<uint8_t>(frame.data, frame.data + frame.can_dlc);
}

void DMCANDevice::callback(const can_frame& frame) {
    std::vector<uint8_t> data = get_data_from_frame(frame);

    switch (callback_mode_) {
        case STATE:
            if (frame.can_dlc >= 8) {
                // Convert frame data to vector and let Motor handle parsing
                StateResult result = CanPacketDecoder::parse_motor_state_data(motor_, data);
                if (frame.can_id == motor_.get_recv_can_id() && result.valid) {
                    motor_.update_state(result.position, result.velocity, result.torque,
                                        result.t_mos, result.t_rotor);
                }
                else {
                    std::cout <<"Callback mode [STATE] : invalid message: " << frame.can_id << std::endl;
                }
            }
            break;
        case PARAM: {
                ParamResult result = CanPacketDecoder::parse_motor_param_data(data);
                if (result.valid) {
                    motor_.set_temp_param(result.rid, result.value);
                } else {
                    std::cout << "Callback mode [PARAM] : invalid message: " << frame.can_id
                              << std::endl;
                }
            break;
        }
        case IGNORE:
            std::cout <<"Callback mode [IGNORE] message : can_id: " << frame.can_id << std::endl;
            return;
        default:
            break;
    }
}
can_frame_ex DMCANDevice::create_can_frame(canid_t send_can_id, std::vector<uint8_t> data) {
    can_frame_ex frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.FrameHeader = data.size();
    frame.FrameId = send_can_id;
    std::copy(data.begin(), data.end(), frame.data);
    return frame;
}

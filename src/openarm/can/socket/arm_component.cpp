#include <openarm/can/socket/arm_component.hpp>

namespace openarm::can::socket {

ArmComponent::ArmComponent(canbus::CANSocket_Ex& can_socket)
    : damiao_motor::DMDeviceCollection(can_socket) {}

void ArmComponent::init_motor_devices(const std::vector<damiao_motor::MotorType>& motor_types,
                                      const std::vector<canid_t>& send_can_ids,
                                      const std::vector<canid_t>& recv_can_ids) {
    // Reserve space to prevent vector reallocation that would invalidate motor
    // references
    motors_.reserve(motor_types.size());

    for (size_t i = 0; i < motor_types.size(); i++) {
        // First, create and store the motor in the vector
        motors_.emplace_back(motor_types[i], send_can_ids[i], recv_can_ids[i]);
        // Then create the device with a reference to the stored motor
        auto motor_device =
            std::make_shared<damiao_motor::DMCANDevice>(motors_.back(), CAN_SFF_MASK);
        get_device_collection().add_device(motor_device);
    }
}

}  // namespace openarm::can::socket

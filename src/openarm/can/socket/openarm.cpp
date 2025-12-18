//#include <linux/can.h>
//#include <linux/can/raw.h>

#include <iostream>
#include <openarm/can/socket/openarm.hpp>

namespace openarm::can::socket {

OpenArm::OpenArm(const std::string& can_interface)
    : can_interface_(can_interface){
    can_socket_ = std::make_unique<canbus::CANSocket>(can_interface_);
    master_can_device_collection_ = std::make_unique<canbus::CANDeviceCollection>(*can_socket_);
    arm_ = std::make_unique<ArmComponent>(*can_socket_);
    gripper_ = std::make_unique<GripperComponent>(*can_socket_);
}

void OpenArm::init_arm_motors(const std::vector<damiao_motor::MotorType>& motor_types,
                              const std::vector<uint32_t>& send_can_ids,
                              const std::vector<uint32_t>& recv_can_ids) {
    if (motor_types.size() != send_can_ids.size() || motor_types.size() != recv_can_ids.size()) {
        throw std::invalid_argument(
            "Motor types, send CAN IDs, and receive CAN IDs vectors must have the same size, "
            "currently: " +
            std::to_string(motor_types.size()) + ", " + std::to_string(send_can_ids.size()) + ", " +
            std::to_string(recv_can_ids.size()));
    }
    arm_->init_motor_devices(motor_types, send_can_ids, recv_can_ids);
    register_dm_device_collection(*arm_);
}

void OpenArm::init_gripper_motor(damiao_motor::MotorType motor_type, uint32_t send_can_id,
                                 uint32_t recv_can_id) {
    gripper_->init_motor_device(motor_type, send_can_id, recv_can_id);
    register_dm_device_collection(*gripper_);
}

void OpenArm::register_dm_device_collection(damiao_motor::DMDeviceCollection& device_collection) {
    for (const auto& [id, device] : device_collection.get_device_collection().get_devices()) {
        master_can_device_collection_->add_device(device);
    }
    sub_dm_device_collections_.push_back(&device_collection);
}

void OpenArm::enable_all() {
    for (damiao_motor::DMDeviceCollection* device_collection : sub_dm_device_collections_) {
        device_collection->enable_all();
    }
}

void OpenArm::set_zero_all() {
    for (damiao_motor::DMDeviceCollection* device_collection : sub_dm_device_collections_) {
        device_collection->set_zero_all();
    }
}

void OpenArm::refresh_all() {
    for (damiao_motor::DMDeviceCollection* device_collection : sub_dm_device_collections_) {
        device_collection->refresh_all();
    }
}

void OpenArm::refresh_one(int i) {
    for (damiao_motor::DMDeviceCollection* device_collection : sub_dm_device_collections_) {
        device_collection->refresh_one(i);
    }
}

void OpenArm::disable_all() {
    for (damiao_motor::DMDeviceCollection* device_collection : sub_dm_device_collections_) {
        device_collection->disable_all();
    }
}
//TODO::Need to know whether all node response are received, introduce transactionId for each writing operation.
void OpenArm::recv_all(int timeout_us) {
    // The timeout for select() is set to timeout_us (default: 500 us).
    // Tuning this value may improve the performance but should be done with caution.

    // CAN 2.0
    {
//        can_frame_ex response_frame;
//        while (can_socket_->is_data_available(timeout_us) &&
//               can_socket_->read_can_frame(response_frame)) {
//            master_can_device_collection_->dispatch_frame_callback(response_frame);
//        }
    }
    // }
}

void OpenArm::query_param_all(int RID) {
    for (damiao_motor::DMDeviceCollection* device_collection : sub_dm_device_collections_) {
        device_collection->query_param_all(RID);
    }
}

void OpenArm::set_callback_mode_all(damiao_motor::CallbackMode callback_mode) {
    for (damiao_motor::DMDeviceCollection* device_collection : sub_dm_device_collections_) {
        device_collection->set_callback_mode_all(callback_mode);
    }
}

}  // namespace openarm::can::socket

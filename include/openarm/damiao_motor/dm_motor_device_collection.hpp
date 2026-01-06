#pragma once
#include <memory>
#include <vector>
#include "../canbus/can_device_collection.hpp"
#include "dm_motor_control.hpp"
#include "dm_motor_device.hpp"

namespace openarm::damiao_motor {
class DMDeviceCollection {
public:
    DMDeviceCollection(canbus::CANSocket& can_socket);
    virtual ~DMDeviceCollection() = default;
    // Common motor operations
    void enable_all();
    void disable_all();
    void set_callback_mode_all(CallbackMode callback_mode);

    // Flash new zero position
    void set_zero(int i);
    void set_zero_all();

    // Refresh operations (for individual motors)
    void refresh_one(int i);
    void refresh_all();

    // Query parameter operations
    void query_param_one(int i, int RID);
    void query_param_all(int RID);

    // MIT control operations
    void mit_control_one(int i, const MITParam& mit_param);
    void mit_control_all(const std::vector<MITParam>& mit_params);

    // PosVel control operation
    void posvel_control_one(int i, const PosVelParam& posvel_param);
    void posvel_control_all(const std::vector<PosVelParam>& posvel_params);

    // Device collection access
    std::vector<Motor> get_motors() const;
    Motor get_motor(int i) const;
    canbus::CANDeviceCollection& get_device_collection() { return *device_collection_; }

protected:
    canbus::CANSocket& can_socket_;
    std::unique_ptr<CanPacketEncoder> can_packet_encoder_;
    std::unique_ptr<CanPacketDecoder> can_packet_decoder_;
    std::unique_ptr<canbus::CANDeviceCollection> device_collection_;

    // Helper methods for subclasses
    void send_command_to_device(std::shared_ptr<DMCANDevice> dm_device, const CANPacket& packet);
    std::vector<std::shared_ptr<DMCANDevice>> get_dm_devices() const;
};
}  // namespace openarm::damiao_motor

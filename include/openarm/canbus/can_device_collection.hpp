#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include "common.h"

namespace openarm::canbus {
class CANDevice;
class CANSocket;
class CANDeviceCollection {
public:
    CANDeviceCollection(canbus::CANSocket& can_socket);
    ~CANDeviceCollection();

    void add_device(const std::shared_ptr<CANDevice>& device);
    void remove_device(const std::shared_ptr<CANDevice>& device);
    void dispatch_frame_callback(const can_frame& frame);
    const std::map<canid_t, std::shared_ptr<CANDevice>>& get_devices() const { return devices_; }
    std::shared_ptr<CANDevice> get_device_by_id(canid_t id) const {
        auto element = devices_.find(id);
        if( element != devices_.end() ) {
            return element->second;
        }
        else {
            return nullptr;
        }
    }
private:
    canbus::CANSocket& can_socket_;
    std::map<canid_t, std::shared_ptr<CANDevice>> devices_;
};
}  // namespace openarm::canbus

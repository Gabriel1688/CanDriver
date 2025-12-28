#pragma once
#include <cstdint>
#include <vector>
#include "common.h"
namespace openarm::canbus {
class CANDevice {
public:
    explicit CANDevice(canid_t send_can_id, canid_t recv_can_id, canid_t recv_can_mask)
        : send_can_id_(send_can_id),
          recv_can_id_(recv_can_id),
          recv_can_mask_(recv_can_mask) {}
    virtual ~CANDevice() = default;

    virtual void callback(const can_frame& frame) = 0;

    canid_t get_send_can_id() const { return send_can_id_; }
    canid_t get_recv_can_id() const { return recv_can_id_; }
    canid_t get_recv_can_mask() const { return recv_can_mask_; }

protected:
    canid_t send_can_id_;
    canid_t recv_can_id_;
    // mask for receiving
    canid_t recv_can_mask_ =0x000007FFU; // CAN_SFF_MASK;
};
}  // namespace openarm::canbus

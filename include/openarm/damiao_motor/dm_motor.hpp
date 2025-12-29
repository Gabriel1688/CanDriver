
#pragma once

#include <cstdint>
#include <cstring>
#include <map>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "dm_motor_constants.hpp"

namespace openarm::damiao_motor {
class Motor {
    friend class DMCANDevice;  // Allow MotorDeviceCan to access protected
                               // members
    friend class DMControl;

public:
    // Constructor
    Motor(MotorType motor_type, uint32_t send_can_id, uint32_t recv_can_id);

    // State getters
    double get_position() const { return state_q_; }
    double get_velocity() const { return state_dq_; }
    double get_torque() const { return state_tau_; }
    int get_state_tmos() const { return state_tmos_; }
    int get_state_trotor() const { return state_trotor_; }

    // Motor property getters
    uint32_t get_send_can_id() const { return send_can_id_; }
    uint32_t get_recv_can_id() const { return recv_can_id_; }
    MotorType get_motor_type() const { return motor_type_; }

    // Enable status getters
    bool is_enabled() const { return enabled_; }
    bool wait_response();

    // Parameter methods
    double get_param(int RID) const;

    // Static methods for motor properties
    static LimitParam get_limit_param(MotorType motor_type);

protected:
    // State update methods
    void update_state(double q, double dq, double tau, int tmos, int trotor);
    void set_state_tmos(int tmos);
    void set_state_trotor(int trotor);
    void set_enabled(bool enabled);
    void set_temp_param(int RID, int val);
    void notify();

    // Motor identifiers
    uint32_t send_can_id_;
    uint32_t recv_can_id_;
    MotorType motor_type_;

    // Enable status
    bool enabled_;

    // Current state
    double state_q_, state_dq_, state_tau_;
    int state_tmos_, state_trotor_;

    // Parameter storage
    std::map<int, double> temp_param_dict_;
    //TODO fix the compilation error:  deleted function.
    //     mutex/conditional variable should not be share pointer.
    std::shared_ptr<std::mutex> request_mutex_;
    std::shared_ptr<std::condition_variable> request_cv_;
    std::shared_ptr<std::atomic<bool>> completed_;
};
}  // namespace openarm::damiao_motor

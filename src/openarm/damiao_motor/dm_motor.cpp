#include <openarm/damiao_motor/dm_motor.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <stdexcept>
#include <string>
#include "spdlog/spdlog.h"
namespace openarm::damiao_motor {

// Constructor
Motor::Motor(MotorType motor_type, uint32_t send_can_id, uint32_t recv_can_id)
    : send_can_id_(send_can_id),
      recv_can_id_(recv_can_id),
      motor_type_(motor_type),
      enabled_(false),
      state_q_(0.0),
      state_dq_(0.0),
      state_tau_(0.0),
      state_tmos_(0),
      state_trotor_(0) {
    completed_ = true;
    request_mutex_ = std::make_shared<std::mutex>();
    request_cv_ = std::make_shared<std::condition_variable>() ;
}

// Enable methods
void Motor::set_enabled(bool enable) { this->enabled_ = enable; }

// Parameter methods
// TODO: storing temp params in motor object might not be a good idea
// also -1 is not a good default value, consider using a different value
double Motor::get_param(int RID) const {
    auto it = temp_param_dict_.find(RID);
    return (it != temp_param_dict_.end()) ? it->second : -1;
}

void Motor::set_temp_param(int RID, int val) {
    temp_param_dict_[RID] = val;
    notify();
}

// State update methods
void Motor::update_state(double q, double dq, double tau, int tmos, int trotor) {
    state_q_ = q;
    state_dq_ = dq;
    state_tau_ = tau;
    state_tmos_ = tmos;
    state_trotor_ = trotor;
}
bool Motor::wait_response() {
    std::unique_lock<std::mutex> lock(*request_mutex_);
    completed_ = false;
    bool result = request_cv_->wait_for(lock, std::chrono::milliseconds{200}, [&] { return completed_; });
    if(result == false) {
        spdlog::error("Motor::wait_response failed to get response within 200ms.");
    }
    return  result;
}
void Motor::notify() {
    std::unique_lock<std::mutex> lock(*request_mutex_);
    if(completed_ == false) {
        completed_ = true;
        request_cv_->notify_one();
    }
}
void Motor::set_state_tmos(int tmos) { state_tmos_ = tmos; }

void Motor::set_state_trotor(int trotor) { state_trotor_ = trotor; }

// Static methods
LimitParam Motor::get_limit_param(MotorType motor_type) {
    size_t index = static_cast<size_t>(motor_type);
    if (index >= MOTOR_LIMIT_PARAMS.size()) {
        throw std::invalid_argument("Invalid motor type: " +
                                    std::to_string(static_cast<int>(motor_type)));
    }
    return MOTOR_LIMIT_PARAMS[index];
}

}  // namespace openarm::damiao_motor

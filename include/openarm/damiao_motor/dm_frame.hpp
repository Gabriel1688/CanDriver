#pragma once

#include <stdint.h>
#include <stddef.h>

extern "C" {

/*********CAN ID Defines***********/
const int CMD_API_COMMON                 = 0x0000;   // General command and MIT mode control frame
const int CMD_API_POS_VELOCITY_MODE      = 0x0100;   // Position and Velocity mode control frame
const int CMD_API_VELOCITY_MODE          = 0x0200;   // Velocity mode control frame

//https://github.com/Gabriel1688/dummy/blob/403d62d1c7ea570c233f986e2b6118dd3a53d0c8/firmware/dummy-35motor-fw/UserApp/protocols/interface_can.cpp

#define PACKED __attribute__((__packed__))

struct PACKED dataframe_clear_error_t {
    uint8_t rsvd[8];
    constexpr dataframe_clear_error_t() : rsvd{0xff, 0xff, 0xff,0xff, 0xff, 0xff,0xff, 0xfb}   {
    }
};

struct PACKED dataframe_enable_motor_t {
    uint8_t rsvd[8];
    constexpr dataframe_enable_motor_t() : rsvd{0xff, 0xff, 0xff,0xff, 0xff, 0xff,0xff, 0xfc}   {
    }
};


struct PACKED dataframe_disable_motor_t {
    uint8_t rsvd[8];
    constexpr dataframe_disable_motor_t() : rsvd{0xff, 0xff, 0xff,0xff, 0xff, 0xff,0xff, 0xfd}   {
    }
};


struct PACKED dataframe_set_zero_position_t {
    uint8_t rsvd[8];
    constexpr dataframe_set_zero_position_t() : rsvd{0xff, 0xff, 0xff,0xff, 0xff, 0xff,0xff, 0xfe}   {
    }
};

typedef struct PACKED {
    int16_t id ;
    uint8_t rsvd[6];
} dataframe_get_motor_status_t;

typedef struct PACKED {
    int16_t id ;
    uint8_t cmd[2];
    uint8_t rsvd[4];
} dataframe_read_motor_param_t;

typedef struct PACKED {
    int16_t id ;
    uint8_t rsvd[6];
} dataframe_save_motor_param_t;

typedef struct PACKED {
    int16_t id ;
    uint8_t rsvd[2];
    uint8_t data[4];
} dataframe_write_motor_param_t;

typedef struct PACKED {
    int16_t p_des ;
    int16_t v_des : 12;
    int16_t    kp : 12;
    int16_t    kd : 12;
    int16_t   tff : 12;
} dataframe_set_mit_set_point_t;

typedef struct PACKED {
    float reserved ;
    float v_des ;
} dataframe_set_vel_set_point_t;

typedef struct PACKED {
    float p_des ;
    float v_des ;
} dataframe_set_pos_vel_set_point_t;

typedef struct PACKED {
    float p_des ;
    int16_t v_des ;
    int16_t i_des ;
} dataframe_pose_with_torque_param_t;

typedef struct PACKED {
    uint8_t id : 4;
    uint8_t err : 4;
    int16_t position ;
    int16_t volecity : 12;
    int16_t tff : 12;
    uint8_t t_mos;
    uint8_t t_rotor;
} dataframe_feedback_t;

union dataframe_t {
    dataframe_clear_error_t             clearError;
    dataframe_enable_motor_t            enableMotor;
    dataframe_disable_motor_t           disableMotor;
    dataframe_set_zero_position_t       setZeroPosition;
    dataframe_set_mit_set_point_t       setMitSetPoint;
    dataframe_set_pos_vel_set_point_t   setPosVelSetPoint;
    dataframe_set_vel_set_point_t       setVelSetPoint;
    dataframe_get_motor_status_t        getMotorStatus;
    dataframe_feedback_t                feedback;
    dataframe_save_motor_param_t        saveMotorParam;
    dataframe_write_motor_param_t       writeMotorParam;
    dataframe_read_motor_param_t        readMotorParam;
    dataframe_pose_with_torque_param_t  poseWithTorqueParam;
    constexpr dataframe_t(dataframe_clear_error_t clearError) : clearError(clearError) {};
    constexpr dataframe_t(dataframe_enable_motor_t enableMotor) : enableMotor(enableMotor) {};
    constexpr dataframe_t(dataframe_disable_motor_t disableMotor) : disableMotor(disableMotor) {};
    constexpr dataframe_t(dataframe_set_zero_position_t setZeroPosition) : setZeroPosition(setZeroPosition) {};

    dataframe_t(dataframe_set_mit_set_point_t setMitSetPoint) : setMitSetPoint(setMitSetPoint) {};
    dataframe_t(dataframe_set_pos_vel_set_point_t setPosVelSetPoint) : setPosVelSetPoint(setPosVelSetPoint) {};
    dataframe_t(dataframe_set_vel_set_point_t setVelSetPoint) : setVelSetPoint(setVelSetPoint) {};
    dataframe_t(dataframe_get_motor_status_t getMotorStatus) : getMotorStatus(getMotorStatus) {};
    dataframe_t(dataframe_feedback_t feedback) : feedback(feedback) {};
    dataframe_t(dataframe_save_motor_param_t saveMotorParam) : saveMotorParam(saveMotorParam) {};
    dataframe_t(dataframe_write_motor_param_t writeMotorParam) : writeMotorParam(writeMotorParam) {};
    dataframe_t(dataframe_read_motor_param_t readMotorParam) : readMotorParam(readMotorParam) {};
    dataframe_t(dataframe_pose_with_torque_param_t poseWithTorqueParam) : poseWithTorqueParam(poseWithTorqueParam) {};

    dataframe_t() {};
    uint8_t data[8];
};

typedef enum {
    deviceBroadcast=0,
    armController,
    gripperController,
    motionController,
    gyroSensor,
    firmwareUpdate=31
} deviceType_t;

typedef enum {
    manufacturerBroadcast=0,
    DAMIAO  = 1,
    DUMMY   = 2,
    TeamUse = 8
} manufacturer_t;

typedef struct PACKED {
    uint16_t                deviceNumber:6;
    uint16_t                api:10;
    manufacturer_t          manufacturer:8;
    deviceType_t            deviceType:5;
    uint8_t                 rsvd:3; //these are DNC
} frameIDFields_t;

typedef union {
    frameIDFields_t         fields;
    uint32_t                raw;
} frameID_t;

typedef struct {
    frameID_t             id;
    dataframe_t           dataframe;
    size_t                length;
} motor_frame_t;
#undef PACKED
} //extern "C"

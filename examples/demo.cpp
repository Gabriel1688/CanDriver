//TODO:: 1. Add CLI interface for testing.
//       2. Add config file for can / ethernet parameters
//       3. Add log to capture the data transaction.
//       4. Resolve the TO-DO/limitation in the code.
//       5. Change the namespace and comments.
//       6. Refine the class, align with FRC CAN implementation.
//       7. Refactor the components implementation, align with FRC 3512 subsystem.
//       8. Introduce timeout / notification and synchronization handling.
//       9. Integrate into the MasterController .

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <thread>

#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"
#include <unistd.h>

int main() {
    try {
        spdlog::info("spdlog version  {}.{}.{}  !", SPDLOG_VER_MAJOR, SPDLOG_VER_MINOR,SPDLOG_VER_PATCH);

        unsigned char data[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34, 0x56, 0x78};

        // Use fmt::join to format the array elements separated by spaces
        // and apply the hexadecimal format specifier to the joined range
        spdlog::info("Raw data in hex: {0:#x}", fmt::join(data, " "));

        std::cout << "=== OpenArm CAN Example ===" << std::endl;
        std::cout << "This example demonstrates the OpenArm API functionality" << std::endl;

        // Initialize OpenArm with CAN interface and enable CAN-FD
        std::cout << "Initializing OpenArm CAN..." << std::endl;
        openarm::can::socket::OpenArm openarm("can0");  // Use CAN-FD on can0 interface

        // Initialize arm motors
        std::vector<openarm::damiao_motor::MotorType> motor_types = {
            openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310};
        std::vector<uint32_t> send_can_ids = {0x01, 0x02};
        std::vector<uint32_t> recv_can_ids = {0x11, 0x12};
        openarm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);

        // Initialize gripper
        std::cout << "Initializing gripper..." << std::endl;
        openarm.init_gripper_motor(openarm::damiao_motor::MotorType::DM4310, 0x08, 0x18);

        // Set callback mode to ignore and enable all motors
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);

        // Enable all motors
        std::cout << "\n=== Enabling Motors ===" << std::endl;
        openarm.enable_all();
        // Allow time (2ms) for the motors to respond for slow operations like enabling
        usleep(200);

        // Set device mode to param and query motor id
        std::cout << "\n=== Querying Motor Recv IDs ===" << std::endl;
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);
        openarm.query_param_all(static_cast<int>(openarm::damiao_motor::RID::MST_ID));

        // Access motors through components
        for (const auto& motor : openarm.get_arm().get_motors()) {
            std::cout << "Arm Motor: " << motor.get_send_can_id() << " ID: "
                      << motor.get_param(static_cast<int>(openarm::damiao_motor::RID::MST_ID))
                      << std::endl;
        }
        for (const auto& motor : openarm.get_gripper().get_motors()) {
            std::cout << "Gripper Motor: " << motor.get_send_can_id() << " ID: "
                      << motor.get_param(static_cast<int>(openarm::damiao_motor::RID::MST_ID))
                      << std::endl;
        }

        // Set device mode to state and control motor
        std::cout << "\n=== Controlling Motors ===" << std::endl;
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        // Control arm motors with position control
        openarm.get_arm().mit_control_all({openarm::damiao_motor::MITParam{2, 1, 0, 0, 0},
                                           openarm::damiao_motor::MITParam{2, 1, 0, 0, 0}});

        // Control arm motors with torque control
        openarm.get_arm().mit_control_all({openarm::damiao_motor::MITParam{0, 0, 0, 0, 0.1},
                                           openarm::damiao_motor::MITParam{0, 0, 0, 0, 0.1}});

        // Control gripper
        std::cout << "Closing gripper..." << std::endl;
        openarm.get_gripper().close();

        for (int i = 0; i < 10; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            openarm.refresh_all();

            // Display arm motor states
            for (const auto& motor : openarm.get_arm().get_motors()) {
                std::cout << "Arm Motor: " << motor.get_send_can_id()
                          << " position: " << motor.get_position() << std::endl;
            }
            // Display gripper state
            for (const auto& motor : openarm.get_gripper().get_motors()) {
                std::cout << "Gripper Motor: " << motor.get_send_can_id()
                          << " position: " << motor.get_position() << std::endl;
            }
        }
        openarm.disable_all();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}

#ifndef LUGGAGE_AV__LUGGAGE_AV_HARDWARE_INTERFACE_HPP_
#define LUGGAGE_AV__LUGGAGE_AV_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"

#include <poll.h>
#include <termios.h>

namespace luggage_av {

class LuggageAVHardwareInterface : public hardware_interface::SystemInterface {
private:
    char* dev_;
    unsigned int baud_;
    pollfd poll_fd_;
    struct termios tty_;

    struct Wheel {
        double ang_vel_min;
        double ang_vel_max;
        int32_t hw_cmd_min;
        int32_t hw_cmd_max;
        uint32_t enc_cpr;

        std::string velocity_command_interface_name;
        std::string position_state_interface_name;
        std::string velocity_state_interface_name;
    };

    Wheel wheels[2];

public:

    // from LifecycleNodeInterface
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state);

    // from SystemInterface
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info);
    // CallbackReturn export_state_interfaces();
    // CallbackReturn export_command_interfaces();
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period);
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period);
    hardware_interface::return_type hardware_read(int32_t* position_left_ptr, int32_t* position_right_ptr, float* velocity_left_ptr, float* velocity_right_ptr);
    hardware_interface::return_type hardware_write(double velocity_left, double velocity_right);

};

}  // namespace luggage_av

#endif  // LUGGAGE_AV__LUGGAGE_AV_HARDWARE_INTERFACE_HPP_

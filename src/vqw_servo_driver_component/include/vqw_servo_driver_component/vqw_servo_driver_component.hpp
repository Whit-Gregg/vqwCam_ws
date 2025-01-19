#ifndef VQW_SERVO_DRIVER_COMPONENT__VQW_SERVO_DRIVER_COMPONENT_HPP_
#define VQW_SERVO_DRIVER_COMPONENT__VQW_SERVO_DRIVER_COMPONENT_HPP_

// #include "vqw_servo_driver_component/i2c_controller.h"
// #include "vqw_servo_driver_component/pca9685.h"
#include "vqw_servo_driver_component/PWMServoDriver.h"
#include "vqw_servo_driver_component/visibility_control.h"
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;

namespace vqw_servo_driver_component
{
    //~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^
    using subscription_SharedPtr = rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr;
    //~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^

    class VqwServoDriverComponent : public rclcpp::Node
    {
      public:
        VqwServoDriverComponent(const rclcpp::NodeOptions &options);
        virtual ~VqwServoDriverComponent();

      private:
        void                      on_msg_recv(std_msgs::msg::Float32MultiArray::UniquePtr msg);
        void                      open_pwm();
        void                      set_pwm(int pwm_index, float pwm_value);
        const std::string         device_name = "/dev/i2c-1";
        int                       i2c_fd      = -1;
        const int                 i2c_address = 0x40;
        const uint8_t prescale = 0;
        subscription_SharedPtr    subscription_;
        I2CController::SharedPtr  i2c_controller;
        PWMServoDriver::SharedPtr pwm_servo_driver;

        using SharedPtr = std::shared_ptr<VqwServoDriverComponent>;
    };       // class VqwServoDriverComponent

    //~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^
    //~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^
}       // namespace vqw_servo_driver_component
//~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^~`-+^

#endif       // VQW_SERVO_DRIVER_COMPONENT__VQW_SERVO_DRIVER_COMPONENT_HPP_

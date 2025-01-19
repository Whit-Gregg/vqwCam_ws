#include "vqw_servo_driver_component/vqw_servo_driver_component.hpp"

namespace vqw_servo_driver_component
{

    VqwServoDriverComponent::VqwServoDriverComponent(const rclcpp::NodeOptions &options) : Node("vqw_servo_driver_component", options)
    {
        i2c_controller = std::make_shared<I2CController>();
        int rc = i2c_controller->open_port(device_name, i2c_address);
        if (rc < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open i2c device: %s   errno=%d  %s", device_name.c_str(), errno, strerror(errno));
            return;
        }

        pwm_servo_driver = std::make_shared<PWMServoDriver>(PCA9685_I2C_ADDRESS, i2c_controller);
        int rc2 = pwm_servo_driver->begin();
        if (!rc2) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize PWM driver");
            return;
        }
        auto topic_callback = [this](std_msgs::msg::Float32MultiArray::UniquePtr msg) -> void {
            // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            this->on_msg_recv(std::move(msg));
        };
        std::string topic_name = "vqw_servo_driver";
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(topic_name, 10, topic_callback);
    }

    VqwServoDriverComponent::~VqwServoDriverComponent()
    {
        //...
        if (i2c_fd >= 0) { close(i2c_fd); }
    }

    void VqwServoDriverComponent::on_msg_recv(std_msgs::msg::Float32MultiArray::UniquePtr msg)
    {
        if (msg->data.size() != 2) {
            RCLCPP_ERROR(this->get_logger(), "Invalid message size: %d,  must be 2", (int)msg->data.size());
            return;
        }
        int pwm_index = static_cast<int>(msg->data[0]+0.49);
        float pwm_value = msg->data[1];
        set_pwm(pwm_index, pwm_value);
    }

    void VqwServoDriverComponent::set_pwm(int pwm_index, float pwm_value)
    {
        pwm_servo_driver->setPWM(pwm_index, 0, pwm_value);
    }


    void VqwServoDriverComponent::open_pwm()
    {
        int rc = i2c_controller->open_port(device_name, i2c_address);
        if (rc < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open i2c device: %s   errno=%d  %s", device_name.c_str(), errno, strerror(errno));
            return;
        }

        

        // i2c_fd = open(device_name.c_str(), O_RDWR);
        // if (i2c_fd < 0) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to open i2c device: %s   errno=%d  %s", device_name.c_str(), errno, strerror(errno));
        //     return;
        // }
        // if (ioctl(i2c_fd, I2C_SLAVE, i2c_address) < 0) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to set i2c address: 0x%x", i2c_address);
        //     close(i2c_fd);
        //     i2c_fd = -1;
        //     return;
        // }

    }

//~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=
}       // namespace vqw_servo_driver_component

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vqw_servo_driver_component::VqwServoDriverComponent)

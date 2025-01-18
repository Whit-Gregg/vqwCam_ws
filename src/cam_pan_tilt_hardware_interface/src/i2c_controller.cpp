#include "cam_pan_tilt_hardware_interface/i2c_controller.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <string.h>
#include <rclcpp/rclcpp.hpp>


I2CController::I2CController(const std::string& _dev_name)
    :status(Status::NOT_OPENED)
{
    // // std::stringstream ss;
    // // ss << channel_name_form << _channel_num;
    //channel = ss.str();
    dev_name = _dev_name;
    open_port();
}
I2CController::~I2CController()
{
    close_port();
}
int I2CController::open_port()
{
    port_handle = open(dev_name.c_str(),O_RDWR);
    if (port_handle < 0)
    {
        //std::cout<<"Failed to open\n";
        RCLCPP_ERROR(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "I2CController::open_port() Failed. dev_name=%s errno=%d  %s", dev_name.c_str(), errno, strerror(errno));
        return -1;
    }
    status = Status::IDLE;
    //std::cout<<"Successed to open the port\n";
    return 1;   
}

int I2CController::close_port()
{
    if (status == Status::NOT_OPENED)
    {
        //std::cout << "No need to close\n";
        RCLCPP_INFO(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "I2CController::close_port() No need to close.");
        return -1;
    }
    wait_for_being_idle();
    //std::cout << "Closing the port...\n";
    close(port_handle);
    status = Status::NOT_OPENED;
    return 1;
}

void I2CController::wait_for_being_idle()
{
    if (status != Status::BUSY)
    {
        return;
    }
    //std::cout << "Wait for work be finished\n";
    int count = 0;
    while ((status == Status::BUSY)&&(count++<1000))
    {
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
}

int I2CController::set_slave_address(const std::uint8_t& _slave_address)
{
    int rc;
    rc = ioctl(port_handle, I2C_SLAVE, _slave_address);
    if(rc < 0)
    {
        // // // std::cout<<"Failed to set slave_address\n";
        // // // std::cout<<"Recommended: i2cdetect\n";
        RCLCPP_ERROR(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "I2CController::set_slave_address() Failed. errno=%d  %s", errno, strerror(errno));
        return -1;
    }
    // // // std::cout<<"Successed to slave address\n";
    return 1;   
}

int I2CController::write_1_byte(const uint8_t& _a_byte)
{
    if(status == Status::NOT_OPENED)
    {
        RCLCPP_ERROR(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "I2CController::write_1_byte() Port is not opened");
        // // // std::cout<<"Port is not opened\n";
        return -1;
    }
    int result;
    wait_for_being_idle();
    status = Status::BUSY;
    result = write(port_handle, &_a_byte, 1);
    status = Status::IDLE;
    return result;
}


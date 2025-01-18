#include "cam_pan_tilt_hardware_interface/cam_pan_tilt_hardware_interface.hpp"

namespace cam_pan_tilt_hardware_interface
{

    /// Initialization of the hardware interface from data parsed from the robot's URDF.
    /**
     * \param[in] hardware_info structure with data from URDF.
     * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
     * \returns CallbackReturn::ERROR if any error happens or data are missing.
     */
    CallbackReturn CamPanTiltHardwareInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        if (ActuatorInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) { return CallbackReturn::ERROR; }
        //===================================================================================================
        hardware_interface::CallbackReturn rc = get_hardware_parameters(hardware_info);
        //===================================================================================================
        return rc;
    }

    CallbackReturn CamPanTiltHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "CamPanTiltHardwareInterface::on_configure()....");
        if (ActuatorInterface::on_configure(previous_state) != CallbackReturn::SUCCESS) { return CallbackReturn::ERROR; }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CamPanTiltHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "CamPanTiltHardwareInterface::on_activate()....");
        if (ActuatorInterface::on_activate(previous_state) != CallbackReturn::SUCCESS) { return CallbackReturn::ERROR; }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CamPanTiltHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "CamPanTiltHardwareInterface::on_deactivate()....");
        if (ActuatorInterface::on_deactivate(previous_state) != CallbackReturn::SUCCESS) { return CallbackReturn::ERROR; }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CamPanTiltHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "CamPanTiltHardwareInterface::on_cleanup()....");
        if (ActuatorInterface::on_cleanup(previous_state) != CallbackReturn::SUCCESS) { return CallbackReturn::ERROR; }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CamPanTiltHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "CamPanTiltHardwareInterface::on_shutdown()....");
        if (ActuatorInterface::on_shutdown(previous_state) != CallbackReturn::SUCCESS) { return CallbackReturn::ERROR; }
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> CamPanTiltHardwareInterface::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces = ActuatorInterface::on_export_state_interfaces();
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr> CamPanTiltHardwareInterface::on_export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces = ActuatorInterface::on_export_command_interfaces();
        return command_interfaces;
    }

    /// Read the current state values from the actuator.
    /**
     * The data readings from the physical hardware has to be updated
     * and reflected accordingly in the exported state interfaces.
     * That is, the data pointed by the interfaces shall be updated.
     *
     * \param[in] time The time at the start of this control loop iteration
     * \param[in] period The measured time taken by the last control loop iteration
     * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
     */
    hardware_interface::return_type CamPanTiltHardwareInterface::read([[maybe_unused]]const rclcpp::Time &time, [[maybe_unused]]const rclcpp::Duration &period)
    {
        // ToDo: Read the current state values from the actuator, and update the state interfaces.
        return hardware_interface::return_type::OK;
    }

    /// Write the current command values to the actuator.
    /**
     * The physical hardware shall be updated with the latest value from
     * the exported command interfaces.
     *
     * \param[in] time The time at the start of this control loop iteration
     * \param[in] period The measured time taken by the last control loop iteration
     * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
     */
    hardware_interface::return_type CamPanTiltHardwareInterface::write([[maybe_unused]]const rclcpp::Time &time, [[maybe_unused]]const rclcpp::Duration &period)
    {
        // ToDo: Write the current command values to the actuator.
        return hardware_interface::return_type::OK;
    }


    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=
    //-=+~-=+~-=  get_hardware_parameters
    //-=+~-=+~-=
    hardware_interface::CallbackReturn CamPanTiltHardwareInterface::get_hardware_parameters(const hardware_interface::HardwareInfo &hardware_info)
    {
        CallbackReturn ret = CallbackReturn::SUCCESS;
        try       //----------- pan_tilt_i2c_port_name -------------
            {
                i2c_port_name = hardware_info.hardware_parameters.at("pan_tilt_i2c_port_name");
            }
        catch (const std::out_of_range &)
            {
                ret = CallbackReturn::ERROR;
                RCLCPP_ERROR(rclcpp::get_logger("CamPanTiltHardwareInterface"), "CamPanTiltHardwareInterface::get_hardware_parameters() "
                                                                            "'pan_tilt_i2c_port_name' must be defined as a hardware "
                                                                            "parameter in the URDF file.");
            }
        if (ret != CallbackReturn::SUCCESS) { return ret; }

        try       //----------- pan_tilt_channel -------------
            {
                pan_tilt_channel = -1;
                auto channel_number  = hardware_info.hardware_parameters.at("pan_tilt_channel");
                pan_tilt_channel = std::stol(channel_number);
            }
        catch (const std::out_of_range &)
            {
                ret = CallbackReturn::ERROR;
                RCLCPP_ERROR(rclcpp::get_logger("CamPanTiltHardwareInterface"), "CamPanTiltHardwareInterface::get_hardware_parameters() "
                                                                            "'pan_tilt_channel' must be defined as a hardware "
                                                                            "parameter in the URDF file.");
            }
        if (ret != CallbackReturn::SUCCESS) { return ret; }

        // if (info_.sensors.size() > 0) { sensor_name = hardware_info.sensors[0].name; }
        // else
        //     {
        //         RCLCPP_ERROR(rclcpp::get_logger("CamPanTiltHardwareInterface"), "CamPanTiltHardwareInterface::get_hardware_parameters() "
        //                                                                     "'sensor_name' must be defined in the URDF file.");
        //         return CallbackReturn::ERROR;
        //     }

        RCLCPP_INFO(rclcpp::get_logger("CamPanTiltHardwareInterface"),
                    "CamPanTiltHardwareInterface::get_hardware_parameters() "
                    "pan_tilt_i2c_port_name='%s', pan_tilt_channel=%d",
                    i2c_port_name.c_str(), pan_tilt_channel);

        /*
        inside semantic_components/imu_sensor/hpp:
            state_interfaces{0..3] = orientation.x, orientation.y, orientation.z,
        orientation.w state_interfaces{4..6] = angular_velocity.x, angular_velocity.y,
        angular_velocity.z state_interfaces{7..9] = linear_acceleration.x,
        linear_acceleration.y, linear_acceleration.z
        */

        //===================================================================================================
        // RCLCPP_INFO(rclcpp::get_logger("CamPanTiltHardwareInterface"),
        //             "CamPanTiltHardwareInterface::get_hardware_parameters()    "
        //             "HardwareInfo.name='%s', HardwareInfo.type='%s', "
        //             "HardwareInfo.hardware_plugin_name='%s'",
        //             hardware_info.name.c_str(), hardware_info.type.c_str(), hardware_info.hardware_plugin_name.c_str());

        // for (uint s = 0; s < info_.sensors.size(); s++)
        //     {
        //         RCLCPP_INFO(rclcpp::get_logger("CamPanTiltHardwareInterface"),
        //                     "CamPanTiltHardwareInterface::get_hardware_parameters()           "
        //                     "sensor[%d].name='%s', sensor[%d].type='%s', "
        //                     "sensor[%d].state_interfaces.size()=%ld",
        //                     s, info_.sensors[s].name.c_str(), s, info_.sensors[s].type.c_str(), s, info_.sensors[s].state_interfaces.size());

        //         for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
        //             {
        //                 RCLCPP_INFO(rclcpp::get_logger("CamPanTiltHardwareInterface"),
        //                             "CamPanTiltHardwareInterface::get_hardware_parameters()                "
        //                             "sensor[%d].state_interfaces[%d].name='%s', "
        //                             "sensor[%d].state_interfaces[%d].data_type='%s'",
        //                             s, i, info_.sensors[s].state_interfaces[i].name.c_str(), s, i, info_.sensors[s].state_interfaces[i].data_type.c_str());
        //             }
        //     }
        return ret;
    }       // end of: CamPanTiltHardwareInterface::get_hardware_parameters

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~



}       // namespace cam_pan_tilt_hardware_interface



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(cam_pan_tilt_hardware_interface::CamPanTiltHardwareInterface, hardware_interface::ActuatorInterface)

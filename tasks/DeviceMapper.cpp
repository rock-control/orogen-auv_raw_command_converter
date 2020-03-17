/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DeviceMapper.hpp"

using namespace auv_raw_command_converter;

DeviceMapper::DeviceMapper(std::string const& name)
    : DeviceMapperBase(name)
{
}

DeviceMapper::DeviceMapper(std::string const& name, RTT::ExecutionEngine* engine)
    : DeviceMapperBase(name, engine)
{
}

DeviceMapper::~DeviceMapper()
{
}

const InputDeviceConfig& DeviceMapper::findDeviceConfig(const std::string& identifier)
{
    for(unsigned i = 0; i < device_configs.size(); i++)
    {
        if(identifier.find(device_configs[i].device_identifier) != std::string::npos)
            return device_configs[i];
    }
    
    throw UnknownDevice(identifier);
}

void DeviceMapper::updateControlMode(const InputDeviceConfig& config, const std::vector<uint8_t>& buttons)
{
    control_mode = _control_mode.value();

    if(config.button_mapping.control_off >= 0 &&
        config.button_mapping.control_off < (int)buttons.size() &&
        buttons[config.button_mapping.control_off] > 0)
    {
        control_mode = auv_raw_command_converter::ControlOff;
    }
    else if(config.button_mapping.acceleration_override >= 0 &&
        config.button_mapping.acceleration_override < (int)buttons.size() &&
        buttons[config.button_mapping.acceleration_override] > 0)
    {
        control_mode = auv_raw_command_converter::AccelerationOverride;
    }
    else if(config.button_mapping.control_chain >= 0 &&
        config.button_mapping.control_chain < (int)buttons.size() &&
        buttons[config.button_mapping.control_chain] > 0)
    {
        control_mode = auv_raw_command_converter::ControlChain;
    }
    else if(config.button_mapping.keep_alive >= 0 &&
        config.button_mapping.keep_alive < (int)buttons.size() &&
        buttons[config.button_mapping.keep_alive] > 0)
    {
        control_mode = auv_raw_command_converter::KeepAlive;
    }
    else if(config.button_mapping.autonomous >= 0 &&
        config.button_mapping.autonomous < (int)buttons.size() &&
        buttons[config.button_mapping.autonomous] > 0)
    {
        control_mode = auv_raw_command_converter::Autonomous;
    }

    _control_mode.set(control_mode);
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See DeviceMapper.hpp for more detailed
// documentation about them.

bool DeviceMapper::configureHook()
{
    if (! DeviceMapperBase::configureHook())
        return false;
    
    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;
    
    device_configs = _device_configs.value();
    control_mode = _control_mode.value();

    scaling.block(0,0,3,1) = _scalings.value().linear;
    scaling.block(3,0,3,1) = _scalings.value().angular;
    
    if(device_configs.empty())
    {
        RTT::log(RTT::Error) << "No device configuration available." << RTT::endlog();
        return false;
    }
    
    for(unsigned i = 0; i < device_configs.size(); i++)
    {
        if(!device_configs[i].isValid())
        {
            RTT::log(RTT::Error) << "Device config contains matrices of an invalid size." << RTT::endlog();
            RTT::log(RTT::Error) << "Every matrix is supposed to be a 6 by N matrix." << RTT::endlog();
            return false;
        }
    }

    linear_angular_cmd = base::commands::LinearAngular6DCommand();
    cmd_timeout = base::Timeout(_cmd_timeout.value());
    
    return true;
}
bool DeviceMapper::startHook()
{
    if (! DeviceMapperBase::startHook())
        return false;
    return true;
}
void DeviceMapper::updateHook()
{
    new_state = RUNNING;
    DeviceMapperBase::updateHook();

    controldev::RawCommand cmd;
    base::Vector6d cmd_out = base::Vector6d::Ones() * base::NaN<double>();
    if(_raw_command.readNewest(cmd) == RTT::NewData)
    {
        try
        {
            const InputDeviceConfig& config = findDeviceConfig(cmd.deviceIdentifier);

            // update control mode from buttons
            updateControlMode(config, cmd.buttonValue);

            linear_angular_cmd.time = cmd.time;
            cmd_timeout.restart();

            // parse input cmd
            unsigned known_fields = std::min(cmd.axisValue.size(), (size_t)config.axis_mapping.cols());
            if(known_fields == 0)
            {
                RTT::log(RTT::Error) << "Unexpected input." << RTT::endlog();
                new_state = UNEXPECTED_INPUT;
            }
            else
            {
                // compute output command
                Eigen::VectorXd cmd_in_v = Eigen::Map<Eigen::VectorXd>(cmd.axisValue.data(), cmd.axisValue.size());
                cmd_out = config.axis_mapping.block(0, 0, 6, known_fields) * cmd_in_v.block(0, 0, known_fields, 1);

                if(control_mode == auv_raw_command_converter::ControlChain)
                {
                    // apply coefficient wise scaling
                    cmd_out.array() = cmd_out.array() * scaling.array();
                }
                else if(control_mode == auv_raw_command_converter::AccelerationOverride)
                {
                    // apply acceleration override scaling
                    cmd_out = cmd_out * _scalings.value().acceleration_override;
                }
            }
        }
        catch (const UnknownDevice& e)
        {
            RTT::log(RTT::Error) << e.what() << RTT::endlog();
            new_state = UNKNOWN_INPUT_DEVICE;
        }
    }

    control_mode = _control_mode.value();
    if(control_mode != auv_raw_command_converter::Autonomous && cmd_timeout.elapsed())
    {
        // control input timeout occurred
        control_mode = auv_raw_command_converter::Timeout;
    }
    else if(new_state != RUNNING)
    {
        // we are in nan error state
        control_mode = auv_raw_command_converter::Timeout;
    }

    switch(control_mode)
    {
        case auv_raw_command_converter::ControlOff:
            linear_angular_cmd.linear = base::Vector3d::Zero();
            linear_angular_cmd.angular = base::Vector3d::Zero();
            _acc_override_command.write(linear_angular_cmd);
            break;
        case auv_raw_command_converter::AccelerationOverride:
            if(cmd_out.allFinite())
            {
                linear_angular_cmd.linear = cmd_out.block(0,0,3,1);
                linear_angular_cmd.angular = cmd_out.block(3,0,3,1);
            }
            _acc_override_command.write(linear_angular_cmd);
            break;
        case auv_raw_command_converter::ControlChain:
            if(cmd_out.allFinite())
            {
                linear_angular_cmd.linear = cmd_out.block(0,0,3,1);
                linear_angular_cmd.angular = cmd_out.block(3,0,3,1);
            }
            _linear_angular_command.write(linear_angular_cmd);
        case auv_raw_command_converter::KeepAlive:
        case auv_raw_command_converter::Autonomous:
            linear_angular_cmd.linear = base::Vector3d::Ones() * base::NaN<double>();
            linear_angular_cmd.angular = base::Vector3d::Ones() * base::NaN<double>();
            _acc_override_command.write(linear_angular_cmd);
            break;
        case auv_raw_command_converter::Timeout:
        default:
            if(new_state == RUNNING)
                new_state = TIMEOUT;
            break;
    }
    
    // write task state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
    }
}
void DeviceMapper::errorHook()
{
    DeviceMapperBase::errorHook();
}
void DeviceMapper::stopHook()
{
    DeviceMapperBase::stopHook();
}
void DeviceMapper::cleanupHook()
{
    DeviceMapperBase::cleanupHook();
}

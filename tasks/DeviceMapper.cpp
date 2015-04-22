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


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See DeviceMapper.hpp for more detailed
// documentation about them.

bool DeviceMapper::configureHook()
{
    if (! DeviceMapperBase::configureHook())
        return false;
    
    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;
    expected_size = 1;
    
    device_configs = _device_configs.value();
    
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
    if(_raw_command.readNewest(cmd) == RTT::NewData)
    {
	try
	{
	    const InputDeviceConfig& config = findDeviceConfig(cmd.deviceIdentifier);
	    
	    // flatten input vector
	    std::vector<double> cmd_in;
	    cmd_in.reserve(expected_size);
	    for(unsigned i = 0; i < cmd.axisValue.size(); i++)
	    {
		for(unsigned j = 0; j < cmd.axisValue[i].size(); j++)
		{
		    cmd_in.push_back(cmd.axisValue[i][j]);
		}
	    }
	    expected_size = cmd_in.size();
	    
	    unsigned known_fields = std::min(cmd_in.size(), (size_t)config.axis_mapping.cols());
	    if(known_fields == 0)
	    {
		RTT::log(RTT::Error) << "Unexpected input." << RTT::endlog();
		new_state = UNEXPECTED_INPUT;
	    }
	    else
	    {
		// compute output command
		Eigen::VectorXd cmd_in_v = Eigen::Map<Eigen::VectorXd>(cmd_in.data(), cmd_in.size());
		base::Vector6d cmd_out = config.axis_mapping.block(0, 0, 6, known_fields) * cmd_in_v.block(0, 0, known_fields, 1);
		
		// apply coefficient wise scaling
		cmd_out.array() = cmd_out.array() * scaling.array();
		
		// write out command
		base::LinearAngular6DCommand linear_angular_cmd;
		linear_angular_cmd.time = cmd.time;
		linear_angular_cmd.linear = cmd_out.block(0,0,3,1);
		linear_angular_cmd.angular = cmd_out.block(3,0,3,1);
		_linear_angular_command.write(linear_angular_cmd);
	    }
	}
	catch (const UnknownDevice& e)
	{
	    RTT::log(RTT::Error) << e.what() << RTT::endlog();
	    new_state = UNKNOWN_INPUT_DEVICE;
	}
	
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

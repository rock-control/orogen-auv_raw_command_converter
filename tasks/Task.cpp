/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace auv_raw_command_converter;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    
    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;
    
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    new_state = RUNNING;
    TaskBase::updateHook();

    controldev::RawCommand cmd;
    if(_raw_command.readNewest(cmd) == RTT::NewData)
    {
	double surge, sway, heave, roll, pitch, yaw = 0.0;
	if(cmd.deviceIdentifier == "mc20")
	{
	    if(cmd.axisValue.size() >= 2 && cmd.axisValue[0].size() >= 4 && cmd.axisValue[1].size() >= 2)
	    {
		// command fits expectation
		surge = cmd.axisValue[0][2];
		sway = cmd.axisValue[0][3];
		heave = cmd.axisValue[1][0];
		pitch = -cmd.axisValue[1][1];
		yaw = cmd.axisValue[0][1];
	    }
	    else
	    {
		new_state = UNEXPECTED_INPUT;
	    }
	}    
	else
	{
	    // assume the command is provided by a joystick
	    if(cmd.axisValue.size() >= 2 && cmd.axisValue[0].size() >= 3 && cmd.axisValue[1].size() >= 1)
	    {
		// command fits expectation
		surge = cmd.axisValue[0][0];
		sway = -cmd.axisValue[0][1];
		heave = -cmd.axisValue[1][0];
		yaw = -cmd.axisValue[0][2];
	    }
	    else
	    {
		new_state = UNEXPECTED_INPUT;
	    }
	    
	}

	base::LinearAngular6DCommand aligned_velocity;
	aligned_velocity.time = cmd.time;
	aligned_velocity.linear(0) = surge;
	aligned_velocity.linear(1) = sway;
	aligned_velocity.linear(2) = heave;
	aligned_velocity.angular(0) = roll;
	aligned_velocity.angular(1) = pitch;
	aligned_velocity.angular(2) = yaw;
	_aligned_velocity_command.write(aligned_velocity);

    }  
    
    // write task state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
    }
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

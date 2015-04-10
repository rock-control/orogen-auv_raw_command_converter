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
    TaskBase::updateHook();

    controldev::RawCommand cmd;
    if(_raw_command.readNewest(cmd) == RTT::NewData)
    {
	//TODO read out device name and act accordingly
	if(cmd.axisValue.size() < 2 || cmd.axisValue[0].size() < 3 || cmd.axisValue[1].size() < 1)
	    state(UNEXPECTED_INPUT);

	base::LinearAngular6DCommand aligned_velocity;
	aligned_velocity.time = cmd.time;
	aligned_velocity.linear(0) = cmd.axisValue[0][0];
	aligned_velocity.linear(1) = -cmd.axisValue[0][1];
	aligned_velocity.linear(2) = -cmd.axisValue[1][0];
	aligned_velocity.angular(0) = 0;
	aligned_velocity.angular(1) = 0;
	aligned_velocity.angular(2) = -cmd.axisValue[0][2];
	_aligned_velocity_command.write(aligned_velocity);

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

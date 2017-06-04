/*
 * MachineInterface.cpp
 *
 *  Created on: Jun 4, 2017
 *      Author: robosim
 */

#include <rosplan_interface_freiburg/machine_interface.h>

MachineInterface::MachineInterface(const std::string& name, const std::string& log_prefix) :
		name_(name), error_state_("MACHINE_NOT_FOUND"), log_prefix_(log_prefix)
{
	ros::NodeHandle nh;
	info_subscriber_ = nh.subscribe("rcll/machine_info", 10, &MachineInterface::machineCB, this);
}

void MachineInterface::machineCB(const rcll_ros_msgs::MachineInfo::ConstPtr& msg)
{
	machines_ = msg;
	if (full_name_ == "")
	{
		std::transform(name_.begin(), name_.end(), name_.begin(), ::toupper);
		for (const auto& machine: msg->machines)
		{
			if (machine.name.find(name_) != std::string::npos)
			{
				full_name_ = machine.name;
			}
		}
	}
}

void MachineInterface::connect_service_prepare_machine()
{
	ros::NodeHandle nh;
	if (!refbox_prepare_machine_.isValid())
	{
		refbox_prepare_machine_ = nh.serviceClient<rcll_ros_msgs::SendPrepareMachine>("rcll/send_prepare_machine", true);
		ROS_INFO_STREAM(log_prefix_<<"Waiting for ROSPlan service "<<refbox_prepare_machine_.getService());
		refbox_prepare_machine_.waitForExistence();
	}
}

bool MachineInterface::hasData() const
{
	return machines_ != NULL;
}

const std::string& MachineInterface::getMachineState()
{
	for (const auto& machine: machines_->machines)
	{
		if (machine.name == full_name_)
		{
			return machine.state;
		}
	}
	return error_state_;
}

bool MachineInterface::waitForState(const std::string& state, ros::Duration timeout)
{
	ros::Time start = ros::Time::now();
	while(ros::ok() && start+timeout>ros::Time::now())
	{
		ros::spinOnce();
		if (machines_ != NULL)
		{
			if (getMachineState() == state)
			{
				return true;
			}
		}
		ros::Rate(1).sleep();
	}
	return false;
}

bool MachineInterface::sendPrepare(rcll_ros_msgs::SendPrepareMachine& srv, const std::string& initial_state,
		const std::string& desired_state)
{
	waitForState(initial_state);
	srv.request.machine = full_name_;
	connect_service_prepare_machine();
	refbox_prepare_machine_.call(srv);
	if (! srv.response.ok)
	{
		return false;
	}

	return waitForState(desired_state);
}


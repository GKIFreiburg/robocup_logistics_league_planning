/***************************************************************************
// *  rosplan_initial_situation.cpp - Create initial situation from config
 *
 *  Created: Fri Feb 24 17:56:41 2017
 *  Copyright  2017  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <ros/ros.h>

#include <rosplan_action_interface/RPActionInterface.h>
#include <rosplan_interface_freiburg/machine_interface.h>

#define GET_CONFIG(privn, n, path, var)	  \
	if (! privn.getParam(path, var)) {      \
		if (! n.getParam(path, var))					\
			ROS_ERROR_STREAM(log_prefix_<<"can not read param "<<path);  \
	}

class ActionBufferCap : public KCL_rosplan::RPActionInterface
{
public:
	ActionBufferCap()
	{
		ros::NodeHandle nh;

		std::string log_prefix_ = "[BufferCap] ";
		machine1_ = std::make_shared<MachineInterface>("CS1", log_prefix_);
		machine2_ = std::make_shared<MachineInterface>("CS2", log_prefix_);
		dispatch_subscriber_ = nh.subscribe("/kcl_rosplan/action_dispatch", 10, &ActionBufferCap::dispatchCB, this);

		ros::NodeHandle nhpriv("~");
		GET_CONFIG(nhpriv, nh, "desired_machine_state", desired_machine_state_)
	}

	void dispatchCB(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		dispatchCallback(msg);
	}

	virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		//(:durative-action buffer-cap
		//	:parameters (?m - cap_station ?i - cs_input ?o - cs_output)
		rcll_ros_msgs::SendPrepareMachine srv;
		srv.request.cs_operation = rcll_ros_msgs::SendPrepareMachine::Request::CS_OP_RETRIEVE_CAP;
		for (const auto& arg: msg->parameters)
		{
			if(arg.key == "m")
			{
				if(arg.value.find(std::to_string(1)) != std::string::npos)
				{
					machine1_->waitForState(desired_machine_state_);
				}
				else if (arg.value.find(std::to_string(2)) != std::string::npos)
				{
					machine2_->waitForState(desired_machine_state_);
				}
				ROS_ERROR_STREAM(log_prefix_<<"Unexpected machine identifier: "<<arg.value);
				return false;
			}
		}
		return true;
	}

private:
	std::string desired_machine_state_;
	std::string log_prefix_;

	ros::Subscriber dispatch_subscriber_;

	std::shared_ptr<MachineInterface> machine1_;
	std::shared_ptr<MachineInterface> machine2_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "action_buffer_cap");
	ros::NodeHandle n;

	ActionBufferCap action;
	action.runActionInterface();

	return 0;
}

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
#include <rosplan_interface_freiburg/async_action_interface.h>
#include <XmlRpc.h>
#include <rosplan_interface_freiburg/machine_interface.h>

#define GET_CONFIG(privn, n, path, var)	  \
	if (! privn.getParam(path, var)) {      \
		if (! n.getParam(path, var))					\
			ROS_ERROR_STREAM(log_prefix_<<"can not read param "<<path);  \
	}


class ActionWaitForMachineState : public rosplan_interface_freiburg::AsyncActionInterface
{
public:
	ActionWaitForMachineState()
	{
		ros::NodeHandle nh;
		ros::NodeHandle nhpriv("~");
		log_prefix_ = "[WaitMachine] ";
		GET_CONFIG(nhpriv, nh, "desired_machine_state", desired_machine_state_)
		GET_CONFIG(nhpriv, nh, "log_prefix", log_prefix_)
		if (log_prefix_.rfind(" ") != log_prefix_.length())
		{
			log_prefix_ = log_prefix_+" ";
		}

		try
		{
			XmlRpc::XmlRpcValue list;
			GET_CONFIG(nhpriv, nh, "machines", list)
			for (size_t i = 0; i < list.size(); ++i)
			{
				auto m = list[i];
				std::string name = static_cast<std::string>(m);
				ROS_INFO_STREAM(log_prefix_<<"adding machine: "<<name);
				machines_[name] = std::make_shared<MachineInterface>(name, log_prefix_);
			}
		} catch (XmlRpc::XmlRpcException& e)
		{
			ROS_ERROR_STREAM(log_prefix_<<e.getMessage());
			exit(-1);
		}

		dispatch_subscriber_ = nh.subscribe("/kcl_rosplan/action_dispatch", 10, &ActionWaitForMachineState::dispatchCB, this);
	}

	void dispatchCB(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		dispatchCallback(msg);
	}

	virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		//(:durative-action buffer-cap
		//	:parameters (?m - cap_station ?i - cs_input ?o - cs_output)
		for (const auto& arg: msg->parameters)
		{
			if(arg.key == "m")
			{
				if (! machines_[arg.value]->hasData())
				{
					ROS_ERROR_STREAM(log_prefix_<<"No machine data received.");
					return false;
				}
				if (machines_.find(arg.value) == machines_.end())
				{
					ROS_ERROR_STREAM(log_prefix_<<"Unexpected machine identifier: "<<arg.value);
					return false;
				}
				return machines_[arg.value]->waitForState(desired_machine_state_);
			}
		}
		return false;
	}

private:
	std::string desired_machine_state_;

	ros::Subscriber dispatch_subscriber_;
	std::map<std::string, MachineInterface::Ptr> machines_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "action_buffer_cap");
	ros::NodeHandle n;

	ActionWaitForMachineState action;
	action.runActionInterface();

	return 0;
}

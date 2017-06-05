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
#include <rcll_ros_msgs/SendPrepareMachine.h>
#include <rcll_ros_msgs/ProductColor.h>
#include <rcll_ros_msgs/MachineInfo.h>
#include <fawkes_msgs/ExecSkillAction.h>

#include <rosplan_interface_freiburg/machine_interface.h>
#include <actionlib/client/simple_action_client.h>

#include <map>
#include <string>
#include <vector>
#include <list>
#include <algorithm>

#include <mutex>
#include <condition_variable>

#define GET_CONFIG(privn, n, path, var)	  \
	if (! privn.getParam(path, var)) {      \
		if (! n.getParam(path, var))					\
			ROS_ERROR_STREAM(log_prefix_<<"can not read param "<<path);  \
	}

typedef actionlib::SimpleActionClient<fawkes_msgs::ExecSkillAction> SkillerClient;

class ActionInsertCap : public KCL_rosplan::RPActionInterface
{
public:
	ActionInsertCap()
	{
		ros::NodeHandle nh;

		skiller_client_ = std::make_shared<SkillerClient>(nh, "skiller", /* spin thread */ false);
		std::string log_prefix_ = "[InserCap] ";
		machines_["cs1"] = std::make_shared<MachineInterface>("cs1", log_prefix_);
		machines_["cs2"] = std::make_shared<MachineInterface>("cs2", log_prefix_);
		dispatch_subscriber_ = nh.subscribe("/kcl_rosplan/action_dispatch", 10, &ActionInsertCap::dispatchCB, this);

		ros::NodeHandle nhpriv("~");
		GET_CONFIG(nhpriv, nh, "initial_machine_state", initial_machine_state_)
		GET_CONFIG(nhpriv, nh, "desired_machine_state", desired_machine_state_)
	}

	void dispatchCB(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		dispatchCallback(msg);
	}

	MachineInterface::Ptr getMachine(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		for (const auto& arg: msg->parameters)
		{
			if(arg.key == "m")
			{
				if (machines_.find(arg.value) == machines_.end())
				{
					ROS_ERROR_STREAM(log_prefix_<<"Unexpected machine identifier: "<<arg.value);
					return NULL;
				}
				if (! machines_[arg.value]->hasData())
				{
					ROS_ERROR_STREAM(log_prefix_<<"No machine data received.");
					return NULL;
				}
				return machines_[arg.value];
			}
		}
		return NULL;
	}

	virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		rcll_ros_msgs::SendPrepareMachine srv;
		srv.request.cs_operation = rcll_ros_msgs::SendPrepareMachine::Request::CS_OP_RETRIEVE_CAP;

		MachineInterface::Ptr machine = getMachine(msg);
		if (machine == NULL)
		{
			return false;
		}
		fawkes_msgs::ExecSkillGoal goal;
		machine->getName();
		// TODO: insert real machine and spot strings
		// TODO: error handling
		goal.skillstring = "get_product_from{place='C-CS1', shelf='LEFT'}";
		skiller_client_->sendGoalAndWait(goal);
		machine->sendPrepare(srv, initial_machine_state_, desired_machine_state_);
		goal.skillstring = "bring_product_to{place='C-CS1', side='input'}";
		skiller_client_->sendGoalAndWait(goal);

		return true;
	}

private:
	std::string initial_machine_state_;
	std::string desired_machine_state_;
	std::string log_prefix_;

	ros::Subscriber dispatch_subscriber_;
	std::map<std::string, MachineInterface::Ptr> machines_;
	std::shared_ptr<actionlib::SimpleActionClient<fawkes_msgs::ExecSkillAction> > skiller_client_;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "action_dispense_product");
	ros::NodeHandle n;

	ActionInsertCap action;
	action.runActionInterface();

	return 0;
}

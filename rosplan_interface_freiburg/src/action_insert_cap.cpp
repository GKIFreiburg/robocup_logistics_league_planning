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

class Shelf
{
public:
	Shelf():
		current_spot_(0)
	{
		spots_ = {"LEFT", "MIDDLE", "RIGHT"};
	}
	const std::string& getNextSpot()
	{
		const std::string& spot = spots_[current_spot_];
		current_spot_ = (current_spot_+1) % spots_.size();
		return spot;
	}
private:
	unsigned int current_spot_;
	std::vector<std::string> spots_;
};

class ActionInsertCap : public KCL_rosplan::RPActionInterface
{
public:
	ActionInsertCap()
	{
		ros::NodeHandle nh;

		skiller_client_ = std::make_shared<SkillerClient>(nh, "skiller", /* spin thread */ false);
		std::string log_prefix_ = "[InserCap] ";
		machine_not_found_ = "MACHINE_NOT_FOUND";
		machines_["cs1"] = std::make_shared<MachineInterface>("cs1", log_prefix_);
		machines_["cs2"] = std::make_shared<MachineInterface>("cs2", log_prefix_);
		shelf_spots_["cs1"] = Shelf();
		shelf_spots_["cs2"] = Shelf();
		dispatch_subscriber_ = nh.subscribe("/kcl_rosplan/action_dispatch", 10, &ActionInsertCap::dispatchCB, this);

		ros::NodeHandle nhpriv("~");
		GET_CONFIG(nhpriv, nh, "initial_machine_state", initial_machine_state_)
		GET_CONFIG(nhpriv, nh, "desired_machine_state", desired_machine_state_)
	}

	void dispatchCB(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		dispatchCallback(msg);
	}

	const std::string& getMachine(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		for (const auto& arg: msg->parameters)
		{
			if(arg.key == "m")
			{
				return arg.value;
			}
		}
		return machine_not_found_;
	}

	virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		const std::string& name = getMachine(msg);
		const auto& machine_it = machines_.find(name);
		if (machine_it == machines_.end())
		{
			ROS_ERROR_STREAM(log_prefix_<<"Unexpected machine identifier: "<<name);
			return false;
		}
		MachineInterface::Ptr machine = machine_it->second;
		if (! machine->hasData())
		{
			ROS_ERROR_STREAM(log_prefix_<<"No machine data received.");
			return false;
		}
		fawkes_msgs::ExecSkillGoal goal;
		goal.skillstring = "get_product_from{place='"+machine->getName()+"', shelf='"+shelf_spots_[name].getNextSpot()+"'}";
		{
			const auto& state = skiller_client_->sendGoalAndWait(goal);
			if (state != state.SUCCEEDED)
			{
				ROS_ERROR_STREAM(log_prefix_<<"Skill "<<goal.skillstring<<" did not succeed. state: "<<state.toString());
				return false;
			}
		}
		rcll_ros_msgs::SendPrepareMachine srv;
		srv.request.cs_operation = rcll_ros_msgs::SendPrepareMachine::Request::CS_OP_RETRIEVE_CAP;
		machine->sendPrepare(srv, initial_machine_state_, desired_machine_state_);

		goal.skillstring = "bring_product_to{place='"+machine->getName()+"', side='input'}";
		{
			const auto& state = skiller_client_->sendGoalAndWait(goal);
			if (state != state.SUCCEEDED)
			{
				ROS_ERROR_STREAM(log_prefix_<<"Skill "<<goal.skillstring<<" did not succeed. state: "<<state.toString());
				return false;
			}
		}

		return true;
	}

private:
	std::string initial_machine_state_;
	std::string desired_machine_state_;
	std::string log_prefix_;
	std::string machine_not_found_;

	std::map<std::string, MachineInterface::Ptr> machines_;
	std::map<std::string, Shelf> shelf_spots_;

	std::shared_ptr<actionlib::SimpleActionClient<fawkes_msgs::ExecSkillAction> > skiller_client_;
	ros::Subscriber dispatch_subscriber_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "action_insert_cap");
	ros::NodeHandle n;

	ActionInsertCap action;
	action.runActionInterface();

	return 0;
}

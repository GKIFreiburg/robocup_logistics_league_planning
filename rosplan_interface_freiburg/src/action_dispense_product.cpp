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

std::string log_prefix_;

class MachineInterface
{
public:
	MachineInterface(const std::string& name) :
			name_(name), error_state_("MACHINE_NOT_FOUND")
	{
		ros::NodeHandle nh;
		info_subscriber_ = nh.subscribe("rcll/machine_info", 10, &MachineInterface::machineCB, this);
	}

	void machineCB(const rcll_ros_msgs::MachineInfo::ConstPtr& msg)
	{
		machines_ = msg;
		if (full_name_ == "")
		{
			for (const auto& machine: msg->machines)
			{
				if (machine.name.find(name_) != std::string::npos)
				{
					full_name_ = machine.name;
				}
			}
		}
	}

	void connect_service_prepare_machine()
	{
		ros::NodeHandle nh;
		if (!refbox_prepare_machine_.isValid())
		{
			refbox_prepare_machine_ = nh.serviceClient<rcll_ros_msgs::SendPrepareMachine>("rcll/send_prepare_machine", true);
			ROS_INFO_STREAM(log_prefix_<<"Waiting for ROSPlan service "<<refbox_prepare_machine_.getService());
			refbox_prepare_machine_.waitForExistence();
		}
	}

	const std::string& getMachineState(const std::string& machine)
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

	bool waitForState(const std::string& state, ros::Duration timeout=ros::Duration(30))
	{
		ros::Time start = ros::Time::now();
		while(ros::ok() && start+timeout>ros::Time::now())
		{
			ros::spinOnce();
			if (machines_ != NULL)
			{
				if (getMachineState(full_name_) == state)
				{
					return true;
				}
			}
			ros::Rate(1).sleep();
		}
		return false;
	}

	bool sendPrepare(rcll_ros_msgs::SendPrepareMachine& srv, const std::string& initial_state,
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

private:
	ros::Subscriber info_subscriber_;
	rcll_ros_msgs::MachineInfo::ConstPtr machines_;
	ros::ServiceClient refbox_prepare_machine_;
	std::string name_;
	std::string full_name_;
	std::string error_state_;
};

class ActionDispenseProduct : public KCL_rosplan::RPActionInterface
{
public:
	ActionDispenseProduct()
	{
		ros::NodeHandle nh;

		machine_ = std::make_shared<MachineInterface>("BS");
		dispatch_subscriber_ = nh.subscribe("/kcl_rosplan/action_dispatch", 10, &ActionDispenseProduct::dispatchCB, this);

		ros::NodeHandle nhpriv("~");
		GET_CONFIG(nhpriv, nh, "initial_machine_state", initial_machine_state_)
		GET_CONFIG(nhpriv, nh, "desired_machine_state", desired_machine_state_)
	}

	void dispatchCB(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		dispatchCallback(msg);
	}

	int extractColor(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		//(:durative-action dispense-product
		//	:parameters (?p - product ?s - step)
		// )
		// (dispense-product p10 red_base_p10)
		for (const auto& arg: msg->parameters)
		{
			if (arg.key == "s")
			{
				size_t pos = arg.value.find("_");
				if (arg.value.substr(0, pos) == "red")
				{
					return rcll_ros_msgs::ProductColor::BASE_RED;
				}
				if (arg.value.substr(0, pos) == "black")
				{
					return rcll_ros_msgs::ProductColor::BASE_BLACK;
				}
				if (arg.value.substr(0, pos) == "silver")
				{
					return rcll_ros_msgs::ProductColor::BASE_SILVER;
				}
			}
		}
		return 0;
	}


	virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		rcll_ros_msgs::SendPrepareMachine srv;
		srv.request.bs_side = rcll_ros_msgs::SendPrepareMachine::Request::BS_SIDE_OUTPUT;
		srv.request.bs_base_color = extractColor(msg);
		machine_->sendPrepare(srv, initial_machine_state_, desired_machine_state_);

		return srv.response.ok;
	}

private:
	std::string initial_machine_state_;
	std::string desired_machine_state_;
	std::string log_prefix_;

	ros::Subscriber dispatch_subscriber_;

	std::shared_ptr<MachineInterface> machine_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "action_dispense_product");
	ros::NodeHandle n;
	log_prefix_ = "[DispenseProduct] ";

	ActionDispenseProduct action;
	action.runActionInterface();

	return 0;
}

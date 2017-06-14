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

#include <rcll_ros_msgs/SendPrepareMachine.h>
#include <rcll_ros_msgs/ProductColor.h>
#include <rcll_ros_msgs/MachineInfo.h>
#include <rcll_ros_msgs/RingInfo.h>
#include <fawkes_msgs/ExecSkillAction.h>

#include <rosplan_interface_freiburg/machine_interface.h>
#include <actionlib/client/simple_action_client.h>

#include <map>
#include <string>
#include <vector>
#include <list>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <rosplan_interface_freiburg/async_action_interface.h>

#include <mutex>
#include <condition_variable>

#define GET_CONFIG(privn, n, path, var)	  \
	if (! privn.getParam(path, var)) {      \
		if (! n.getParam(path, var))					\
			ROS_ERROR_STREAM(log_prefix_<<"can not read param "<<path);  \
	}

typedef actionlib::SimpleActionClient<fawkes_msgs::ExecSkillAction> SkillerClient;

class ActionTransportProduct : public rosplan_interface_freiburg::AsyncActionInterface
{
public:
	ActionTransportProduct()
	{
		ros::NodeHandle nh;
		log_prefix_ = "[TransportP] ";
		skiller_client_ = std::make_shared<SkillerClient>(nh, "skiller", /* spin thread */true);
		std::string log_prefix_ = "[TransportProduct] ";
		parameter_not_found_ = "PARAMETER_NOT_FOUND";
		machines_["bs"] = std::make_shared<MachineInterface>("bs", log_prefix_);
		machines_["cs1"] = std::make_shared<MachineInterface>("cs1", log_prefix_);
		machines_["cs2"] = std::make_shared<MachineInterface>("cs2", log_prefix_);
		machines_["rs1"] = std::make_shared<MachineInterface>("rs1", log_prefix_);
		machines_["rs2"] = std::make_shared<MachineInterface>("rs2", log_prefix_);
		machines_["ds"] = std::make_shared<MachineInterface>("ds", log_prefix_);
		dispatch_subscriber_ = nh.subscribe("/kcl_rosplan/action_dispatch", 10, &ActionTransportProduct::dispatchCB, this);
		sub_ring_info_ = nh.subscribe("rcll/ring_info", 10, &ActionTransportProduct::ring_info_cb, this);

		ros::NodeHandle nhpriv("~");
		GET_CONFIG(nhpriv, nh, "initial_machine_state", initial_machine_state_)
		GET_CONFIG(nhpriv, nh, "desired_machine_state", desired_machine_state_)
		GET_CONFIG(nhpriv, nh, "robot_name", robot_name_)

	}

	void dispatchCB(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		for (const auto& arg: msg->parameters)
		{
			if(arg.key == "r" && arg.value == robot_name_)
			{
				dispatchCallback(msg);
			}
		}
	}

	void ring_info_cb(const rcll_ros_msgs::RingInfo::ConstPtr& msg)
	{
		for (const auto& ring: msg->rings)
		{
			ring_colors_materials_[ring.ring_color] = ring.raw_material;
		}
	}

	bool setupPrepareRequest(const std::string& step, rcll_ros_msgs::SendPrepareMachine::Request& request)
	{
		// orange_ring_p10, black_cap_p10, gate1_delivery_p10
		std::vector<std::string> parts;
		boost::split(parts, step, boost::is_any_of("_"));
		if (parts.size() != 3)
		{
			return false;
		}
		const std::string& operation = parts[1];
		const std::string& details = parts[0];
		if (operation == "ring")
		{
			if (details == "blue")
			{
				request.rs_ring_color = rcll_ros_msgs::ProductColor::RING_BLUE;
			}
			else if (details == "green")
			{
				request.rs_ring_color = rcll_ros_msgs::ProductColor::RING_GREEN;
			}
			else if (details == "orange")
			{
				request.rs_ring_color = rcll_ros_msgs::ProductColor::RING_ORANGE;
			}
			else if (details == "yellow")
			{
				request.rs_ring_color = rcll_ros_msgs::ProductColor::RING_YELLOW;
			}
			else
			{
				return false;
			}
		}
		else if (operation == "cap")
		{
			request.cs_operation = request.CS_OP_MOUNT_CAP;
		}
		else if (operation == "delivery")
		{
			request.ds_gate = std::stoi(details.substr(4, 5));
		}
		return true;
	}

	virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		const std::string& name_out = boundParameters["om"];
		const auto& machine_out_it = machines_.find(name_out);
		if (machine_out_it == machines_.end())
		{
			ROS_ERROR_STREAM(log_prefix_<<"Unexpected machine identifier: "<<name_out);
			return false;
		}
		MachineInterface::Ptr machine_out = machine_out_it->second;
		if (! machine_out->hasData())
		{
			ROS_ERROR_STREAM(log_prefix_<<"No machine data received.");
			return false;
		}

		std::string side = "output";
		if (boundParameters["o"].find("in") != std::string::npos)
		{
			side = "input";
		}
		fawkes_msgs::ExecSkillGoal goal;
		goal.skillstring = "get_product_from{place='" + machine_out->getName() + "', side='"+side+"'}";
		{
			ROS_INFO_STREAM(log_prefix_<<"Sending skill "<<goal.skillstring<<"...");
			const auto& state = skiller_client_->sendGoalAndWait(goal, execute_timeout_);
			if (state != state.SUCCEEDED)
			{
				ROS_ERROR_STREAM(log_prefix_<<"Skill "<<goal.skillstring<<" did not succeed. state: "<<state.toString());
				return false;
			}
			ROS_INFO_STREAM(log_prefix_<<"Skill "<<goal.skillstring<<" succeeded");
		}

		const std::string& name = boundParameters["m"];
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

		rcll_ros_msgs::SendPrepareMachine srv;
		setupPrepareRequest(boundParameters["s2"], srv.request);
		ROS_INFO_STREAM(
				log_prefix_<<"sending prepare request, wait for initial state: "<<initial_machine_state_<<", wait for desired state: "<<desired_machine_state_);
		bool success = machine->sendPrepare(srv, initial_machine_state_, desired_machine_state_);
		if (! success)
		{
			ROS_ERROR_STREAM(log_prefix_<<"Send prepare failed.");
			return false;
		}

		goal.skillstring = "bring_product_to{place='"+machine->getName()+"', side='input'}";
		{
			const auto& state = skiller_client_->sendGoalAndWait(goal, execute_timeout_);
			if (state != state.SUCCEEDED)
			{
				ROS_ERROR_STREAM(log_prefix_<<"Skill "<<goal.skillstring<<" did not succeed. state: "<<state.toString());
				return false;
			}
		}

		if (srv.request.rs_ring_color != 0)
		{
			// update material-stored numerical fluent
			rosplan_knowledge_msgs::KnowledgeItem material;
			material.knowledge_type = material.FUNCTION;
			material.attribute_name = "material-stored";
			diagnostic_msgs::KeyValue rs;
			rs.key = "m";
			rs.value = name;
			material.values.push_back(rs);
			lookupNumericalValue(material);
			material.function_value -= ring_colors_materials_[srv.request.rs_ring_color];
			updateNumericalValue(material);
		}
		return true;
	}

private:
	std::string log_prefix_;
	std::string robot_name_;
	std::string initial_machine_state_;
	std::string desired_machine_state_;
	std::string parameter_not_found_;

	std::map<std::string, MachineInterface::Ptr> machines_;
	std::map<int, int> ring_colors_materials_;

	std::shared_ptr<actionlib::SimpleActionClient<fawkes_msgs::ExecSkillAction> > skiller_client_;
	ros::Subscriber dispatch_subscriber_;
	ros::Subscriber sub_ring_info_;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "action_transport_product");
	ros::NodeHandle n;

	ActionTransportProduct action;
	action.runActionInterface();

	return 0;
}

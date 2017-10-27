/***************************************************************************
 *  rosplan_interface_behaviorengine.cpp - Call skills on Lua BE from ROSPlan
 *
 *  Created: Fri Feb 10 13:40:14 2017
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

#include <fawkes_msgs/ExecSkillAction.h>

#include <rosplan_interface_freiburg/machine_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <rosplan_interface_freiburg/async_action_interface.h>

#define GET_CONFIG(privn, n, path, var)	  \
	if (! privn.getParam(path, var)) {      \
		if (! n.getParam(path, var))					\
			ROS_ERROR_STREAM(log_prefix_<<"can not read param "<<path);  \
	}

typedef actionlib::SimpleActionClient<fawkes_msgs::ExecSkillAction> SkillerClient;

class ActionMove : public rosplan_interface_freiburg::AsyncActionInterface
{
public:
	ActionMove()
	{
		ros::NodeHandle nh;

		skiller_client_ = std::make_shared<SkillerClient>(nh, "skiller", /* spin thread */true);
		log_prefix_ = "[Move] ";
		param_not_found_ = "PARAM_NOT_FOUND";
		dispatch_subscriber_ = nh.subscribe("/kcl_rosplan/action_dispatch", 10, &ActionMove::dispatchCB, this);

		ros::NodeHandle nhpriv("~");
		GET_CONFIG(nhpriv, nh, "robot_name", robot_name_)
		GET_CONFIG(nhpriv, nh, "team_color", team_color_)

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

	const std::string& getDestination(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		for (const auto& arg: msg->parameters)
		{
			if(arg.key == "l2")
			{
				return arg.value;
			}
		}
		return param_not_found_;
	}

	virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		const std::string& destination = getDestination(msg);
		size_t pos = destination.find("_");
		if (pos == std::string::npos)
		{
			return false;
		}
		std::string name = team_color_.substr(0,1)+"-"+destination.substr(0, pos)+"-"+destination.substr(pos+1, 1);
		std::transform(name.begin(), name.end(), name.begin(), ::toupper);
		fawkes_msgs::ExecSkillGoal goal;
		goal.skillstring = "ppgoto{place='"+name+"'}";
		{
			const auto& state = skiller_client_->sendGoalAndWait(goal, execute_timeout_);
			if (state != state.SUCCEEDED)
			{
				ROS_ERROR_STREAM(log_prefix_<<"Skill "<<goal.skillstring<<" did not succeed. state: "<<state.toString());
				return false;
			}
		}

		// HACK: do not set robot-recently-moved effect. Can produce deadlock situation!
		std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator pit = op.at_end_add_effects.begin();
		for (; pit != op.at_end_add_effects.end(); pit++)
		{
			if (pit->name == "robot-recently-moved")
			{
				pit = op.at_end_add_effects.erase(pit);
			}
		}

		return true;
	}

private:
	std::string robot_name_;
	std::string team_color_;
	std::string param_not_found_;

	std::shared_ptr<actionlib::SimpleActionClient<fawkes_msgs::ExecSkillAction> > skiller_client_;
	ros::Subscriber dispatch_subscriber_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "action_move");
	ros::NodeHandle n;

	ActionMove action;
	action.runActionInterface();

	return 0;
}

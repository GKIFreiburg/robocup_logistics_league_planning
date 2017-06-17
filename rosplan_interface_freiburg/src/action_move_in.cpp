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
#include <actionlib/client/simple_action_client.h>
#include <rosplan_interface_freiburg/async_action_interface.h>

#define GET_CONFIG(privn, n, path, var)	  \
	if (! privn.getParam(path, var)) {      \
		if (! n.getParam(path, var))					\
			ROS_ERROR_STREAM(log_prefix_<<"can not read param "<<path);  \
	}

typedef actionlib::SimpleActionClient<fawkes_msgs::ExecSkillAction> SkillerClient;

class ActionMoveIn : public rosplan_interface_freiburg::AsyncActionInterface
{
public:
	ActionMoveIn()
	{
		ros::NodeHandle nh;

		skiller_client_ = std::make_shared<SkillerClient>(nh, "skiller", /* spin thread */true);
		log_prefix_ = "[MoveIn] ";
		dispatch_subscriber_ = nh.subscribe("/kcl_rosplan/action_dispatch", 10, &ActionMoveIn::dispatchCB, this);

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

	virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		fawkes_msgs::ExecSkillGoal goal;
		goal.skillstring = "drive_into_field{team='"+team_color_+"', wait=0.0}";
		{
			const auto& state = skiller_client_->sendGoalAndWait(goal, execute_timeout_);
			if (state != state.SUCCEEDED)
			{
				ROS_ERROR_STREAM(log_prefix_<<"Skill "<<goal.skillstring<<" did not succeed. state: "<<state.toString());
				return false;
			}
		}
		return true;
	}

private:
	std::string log_prefix_;
	std::string robot_name_;
	std::string team_color_;

	std::shared_ptr<actionlib::SimpleActionClient<fawkes_msgs::ExecSkillAction> > skiller_client_;
	ros::Subscriber dispatch_subscriber_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "action_move_in");
	ros::NodeHandle n;

	ActionMoveIn action;
	action.runActionInterface();

	return 0;
}

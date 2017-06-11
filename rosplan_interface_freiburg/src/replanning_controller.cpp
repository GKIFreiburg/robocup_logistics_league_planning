/***************************************************************************
 *  rosplan_interface_rcllrefbox.cpp - Referee box actions
 *
 *  Created: Wed Feb 16 22:37:18 2017
 *  Copyright  2017  Tim Niemueller [www.niemueller.de] and Erez Karpas
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

#include <std_msgs/String.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>

#include <rosplan_knowledge_msgs/DomainFormula.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h>
#include <rosplan_knowledge_msgs/GetDomainAttributeService.h>
#include <diagnostic_msgs/KeyValue.h>
#include <rcll_ros_msgs/Order.h>
#include <rcll_ros_msgs/OrderInfo.h>
#include <rcll_ros_msgs/RingInfo.h>
#include <rcll_ros_msgs/MachineInfo.h>
#include <rcll_ros_msgs/ProductColor.h>
#include <rcll_ros_msgs/GameState.h>

#define GET_CONFIG(privn, n, path, var)	  \
	if (! privn.getParam(path, var)) {      \
		if (! n.getParam(path, var))					\
			ROS_ERROR_STREAM(log_prefix_<<"can not read param "<<path);  \
	}

class ReplanningController
{
public:
	typedef std::shared_ptr<ReplanningController> Ptr;
	typedef std::map<int, rosplan_dispatch_msgs::ActionFeedback::ConstPtr> ActionMap;
	ReplanningController(ros::NodeHandle n) :
		n_(n), log_prefix_("[MissionControl] ")
	{
		game_state_sub_ = n_.subscribe("game_state", 10, &ReplanningController::gameStateCB, this);
		planning_state_sub_ = n_.subscribe("/kcl_rosplan/system_state", 10, &ReplanningController::planningStateCB, this);
		action_feedback_sub_ = n_.subscribe("/kcl_rosplan/action_feedback", 10, &ReplanningController::actionFeedbackCB, this);
		planning_command_pub_ = n.advertise<std_msgs::String>("/kcl_rosplan/planning_commands", 10, false);

		wait_for_action_finish_ = false;
		ready_state_ = "Ready";
		planning_state_ = "Planning";
		dispatching_state_ = "Dispatching";
		start_command_.data = "plan";
		cancel_command_.data = "cancel";
		achieved_status_ = "achieved";
		enabled_status_ = "enabled";
		failed_status_ = "failed";

		ros::NodeHandle n_priv("~");
		GET_CONFIG(n_priv, n, "wait_for_action_finish", wait_for_action_finish_)
		GET_CONFIG(n_priv, n, "ready_state", ready_state_)
		GET_CONFIG(n_priv, n, "planning_state", planning_state_)
		GET_CONFIG(n_priv, n, "dispatching_state", dispatching_state_)
		GET_CONFIG(n_priv, n, "start_command", start_command_.data)
		GET_CONFIG(n_priv, n, "cancel_command", cancel_command_.data)
		GET_CONFIG(n_priv, n, "achieved_status", achieved_status_)
		GET_CONFIG(n_priv, n, "enabled_status", enabled_status_)
		GET_CONFIG(n_priv, n, "failed_status", failed_status_)
	}

	void gameStateCB(const rcll_ros_msgs::GameState::ConstPtr& msg)
	{
		latest_game_state_ = msg;
		startPlanningWhenReady();
	}

	void actionFeedbackCB(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg)
	{
		if (msg->status == enabled_status_)
		{
			enabled_actions_[msg->action_id] = msg;
		}
		if (msg->status == achieved_status_ || msg->status == failed_status_)
		{
			enabled_actions_.erase(msg->action_id);
		}
	}

	void planningStateCB(const std_msgs::String::ConstPtr& msg)
	{
		latest_planning_state_ = msg;
		startPlanningWhenReady();
	}

	void startPlanningWhenReady()
	{
		if (!latest_game_state_)
		{
			return;
		}
		if (latest_game_state_->state == rcll_ros_msgs::GameState::STATE_RUNNING
				&& latest_game_state_->phase == rcll_ros_msgs::GameState::PHASE_PRODUCTION)
		{
			if(!latest_planning_state_)
			{
				ROS_WARN_STREAM_THROTTLE(5, log_prefix_<<"Production phase running, but planning system not connected");
				return;
			}
			if (latest_planning_state_->data == ready_state_)
			{
				if (wait_for_action_finish_ && ! enabled_actions_.empty())
				{
					ROS_INFO_STREAM_THROTTLE(5, log_prefix_<<"Waiting for actions to finish...");
					return;
				}
				ROS_INFO_STREAM(log_prefix_<<"Starting planning now.");
				planning_command_pub_.publish(start_command_);
			}
		}
	}

	void cancelDispatch()
	{
		planning_command_pub_.publish(cancel_command_);
	}

private:
	std::string log_prefix_;

	ros::Subscriber game_state_sub_;
	ros::Subscriber action_feedback_sub_;
	ros::Subscriber planning_state_sub_;

	ros::Publisher planning_command_pub_;

	rcll_ros_msgs::GameState::ConstPtr latest_game_state_;
	std_msgs::String::ConstPtr latest_planning_state_;

	std_msgs::String start_command_;
	std_msgs::String cancel_command_;

	std::string ready_state_;
	std::string planning_state_;
	std::string dispatching_state_;

	std::string enabled_status_;
	std::string achieved_status_;
	std::string failed_status_;

	bool wait_for_action_finish_;
	ActionMap enabled_actions_;

	ros::NodeHandle n_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "replanning_controller");

	ros::NodeHandle n;

	ReplanningController replanning(n);

	ros::spin();

	return 0;
}

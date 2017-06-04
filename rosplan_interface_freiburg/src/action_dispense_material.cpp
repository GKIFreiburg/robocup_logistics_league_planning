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
#include <rcll_ros_msgs/ProductColor.h>

#include <rosplan_interface_freiburg/machine_interface.h>

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



class ActionDispenseMaterial : public KCL_rosplan::RPActionInterface
{
public:
	ActionDispenseMaterial()
	{
		ros::NodeHandle nh;

		std::string log_prefix_ = "[DispenseMaterial] ";
		machine_ = std::make_shared<MachineInterface>("BS", log_prefix_);
		dispatch_subscriber_ = nh.subscribe("/kcl_rosplan/action_dispatch", 10, &ActionDispenseMaterial::dispatchCB, this);

		ros::NodeHandle nhpriv("~");
		GET_CONFIG(nhpriv, nh, "initial_machine_state", initial_machine_state_)
		GET_CONFIG(nhpriv, nh, "desired_machine_state", desired_machine_state_)
	}

	void dispatchCB(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		dispatchCallback(msg);
	}

	virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		//(:durative-action dispense-material
		//	:parameters (?m - base_station ?o - bs_output)
		rcll_ros_msgs::SendPrepareMachine srv;
		srv.request.bs_side = rcll_ros_msgs::SendPrepareMachine::Request::BS_SIDE_OUTPUT;
		srv.request.bs_base_color = rcll_ros_msgs::ProductColor::BASE_BLACK;
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
	ros::init(argc, argv, "action_dispense_material");
	ros::NodeHandle n;

	ActionDispenseMaterial action;
	action.runActionInterface();

	return 0;
}

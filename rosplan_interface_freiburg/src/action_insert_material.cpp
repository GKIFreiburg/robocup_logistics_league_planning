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

class ActionInsertMaterial: public rosplan_interface_freiburg::AsyncActionInterface
{
public:
        ActionInsertMaterial()
        {
                ros::NodeHandle nh;

                skiller_client_ = std::make_shared<SkillerClient>(nh, "skiller", /* spin thread */true);
                std::string log_prefix_ = "[InsertMaterial] ";
                parameter_not_found_ = "PARAMETER_NOT_FOUND";
                machines_["bs"] = std::make_shared<MachineInterface>("bs", log_prefix_);
                machines_["rs1"] = std::make_shared<MachineInterface>("rs1", log_prefix_);
                machines_["rs2"] = std::make_shared<MachineInterface>("rs2", log_prefix_);
                machines_["cs1"] = std::make_shared<MachineInterface>("cs1", log_prefix_);
                machines_["cs2"] = std::make_shared<MachineInterface>("cs2", log_prefix_);
                machines_["ds"] = std::make_shared<MachineInterface>("ds", log_prefix_);
                dispatch_subscriber_ = nh.subscribe("/kcl_rosplan/action_dispatch", 10, &ActionInsertMaterial::dispatchCB, this);

                ros::NodeHandle nhpriv("~");
                GET_CONFIG(nhpriv, nh, "robot_name", robot_name_)

        }

        void dispatchCB(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
        {
                for (const auto& arg : msg->parameters)
                {
                        if (arg.key == "r" && arg.value == robot_name_)
                        {
                                dispatchCallback(msg);
                        }
                }
        }

        virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
        {

                fawkes_msgs::ExecSkillGoal goal;


                const std::string& name = boundParameters["m"];
                const auto& machine_it = machines_.find(name);
                if (machine_it == machines_.end())
                {
                        ROS_ERROR_STREAM(log_prefix_<<"Unexpected machine identifier: "<<name);
                        return false;
                }
                MachineInterface::Ptr machine = machine_it->second;
                if (!machine->hasData())
                {
                        ROS_ERROR_STREAM(log_prefix_<<"No machine data received.");
                        return false;
                }

                std::string slide;
                if (name.find("rs") != std::string::npos)
                {
                        slide = ", slide=true";
                }
                else if (name == "ds")
                {
                        // prepare discard
                        rcll_ros_msgs::SendPrepareMachine srv;
                        srv.request.ds_gate = 3;
                        ROS_INFO_STREAM(
                                        log_prefix_<<"sending prepare request, wait for initial state: "<<initial_machine_state_<<", wait for desired state: "<<desired_machine_state_);
                        bool success = machine->sendPrepare(srv, initial_machine_state_, desired_machine_state_);
                        if (!success)
                        {
                                ROS_ERROR_STREAM(log_prefix_<<"Send prepare failed.");
                                return false;
                        }
                }
                goal.skillstring = "bring_product_to{place='" + machine->getName() + "', side='input'" + slide + "}";
                {
                        const auto& state = skiller_client_->sendGoalAndWait(goal, execute_timeout_);
                        if (state != state.SUCCEEDED)
                        {
                                ROS_ERROR_STREAM(log_prefix_<<"Skill "<<goal.skillstring<<" did not succeed. state: "<<state.toString());
                                return false;
                        }
                        //send predicate robot holding material REMOVE
                        rosplan_knowledge_msgs::KnowledgeItem effect;
                        effect.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
                        effect.attribute_name = "robot-holding-material";

                        diagnostic_msgs::KeyValue rs;
                        rs.key = "r";
                        rs.value = robot_name_;
                        effect.values.push_back(rs);

                        sendEffectREMOVE(effect);

                        //send predicate robot-holding-something REMOVE
                        rosplan_knowledge_msgs::KnowledgeItem holding;
                        holding.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
                        holding.attribute_name = "robot-holding-something";

                        rs.key = "r";
                        rs.value = robot_name_;
                        holding.values.push_back(rs);

                        sendEffectREMOVE(holding);

                        //send predicate conveyor-full ADD
                        rosplan_knowledge_msgs::KnowledgeItem conveyor;
                        conveyor.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
                        conveyor.attribute_name = "conveyor-full";

                        const std::string& machine_name = boundParameters["m"];
                        rs.key = "m";
                        rs.value = machine_name;
                        conveyor.values.push_back(rs);

                        sendEffectADD(conveyor);
                }

                // update material-stored numerical fluent
                rosplan_knowledge_msgs::KnowledgeItem material;
                material.knowledge_type = material.FUNCTION;
                material.attribute_name = "material-stored";
                diagnostic_msgs::KeyValue rs;
                rs.key = "m";
                rs.value = name;
                material.values.push_back(rs);
                lookupNumericalValue(material);
                material.function_value += 1;
                updateNumericalValue(material);

                return true;
        }

private:
        std::string log_prefix_;
        std::string robot_name_;
        std::string initial_machine_state_;
        std::string desired_machine_state_;
        std::string parameter_not_found_;

        std::map<std::string, MachineInterface::Ptr> machines_;

        std::shared_ptr<actionlib::SimpleActionClient<fawkes_msgs::ExecSkillAction> > skiller_client_;
        ros::Subscriber dispatch_subscriber_;
};

int main(int argc, char **argv)
{
        ros::init(argc, argv, "action_insert_material");
        ros::NodeHandle n;

        ActionInsertMaterial action;
        action.runActionInterface();

        return 0;
}

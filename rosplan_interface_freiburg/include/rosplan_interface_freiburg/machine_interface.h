/*
 * MachineInterface.h
 *
 *  Created on: Jun 4, 2017
 *      Author: robosim
 */

#ifndef SRC_MACHINEINTERFACE_H_
#define SRC_MACHINEINTERFACE_H_

#include <ros/ros.h>
#include <rcll_ros_msgs/SendPrepareMachine.h>
#include <rcll_ros_msgs/MachineInfo.h>

/**
 * MachineInterface subscribes to the machine info topic and connects to the sendPrepare service.
 */
class MachineInterface
{
public:
	/**
	 * const std::string& name [in]: name of the machine, can be without team prefix. e.g. CS, RS1, CS2, DS
	 */
	MachineInterface(const std::string& name, const std::string& log_prefix);

	/**
	 * Returns the most recent state of this machine
	 */
	const std::string& getMachineState();

	/**
	 * Blocks and waits until machine has state.
	 * const std::string& state [in]: the state to wait for. Possible states include: IDLE, PREPARED, WAITING-AT-OUTPUT
	 */
	bool waitForState(const std::string& state, ros::Duration timeout=ros::Duration(30));

	/**
	 * Sends a prepare service request to the refbox. Machine name is set automatically.
	 * rcll_ros_msgs::SendPrepareMachine& srv [in/out]: contains specific parameters for the request.
	 * const std::string& initial_state [in]: block and wait until machine has initial_state before the request is sent.
	 * const std::string& desired_state [in]: block and wait until machine has desired_state after the request is sent.
	 * returns: true if desired state was achieved after sending the service request
	 */
	bool sendPrepare(rcll_ros_msgs::SendPrepareMachine& srv, const std::string& initial_state,
			const std::string& desired_state);

	void machineCB(const rcll_ros_msgs::MachineInfo::ConstPtr& msg);

	void connect_service_prepare_machine();

private:
	ros::Subscriber info_subscriber_;
	ros::ServiceClient refbox_prepare_machine_;

	rcll_ros_msgs::MachineInfo::ConstPtr machines_;

	std::string name_;
	std::string full_name_;
	std::string error_state_;
	std::string log_prefix_;
};

#endif /* SRC_MACHINEINTERFACE_H_ */

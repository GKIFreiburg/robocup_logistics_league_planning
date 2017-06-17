#ifndef SYNC_ACTION_INTERFACE_H_
#define SYNC_ACTION_INTERFACE_H_

#include <ros/ros.h>
#include <boost/tokenizer.hpp>

#include <rosplan_dispatch_msgs/ActionFeedback.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>
#include <rosplan_knowledge_msgs/DomainFormula.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h>
#include <rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h>
#include <rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h>
#include <diagnostic_msgs/KeyValue.h>

#include <map>

namespace rosplan_interface_freiburg
{

class AsyncActionInterface
{

private:

protected:

	/* PDDL info and publisher */
	std::map<std::string, rosplan_knowledge_msgs::DomainFormula> predicates;
	rosplan_knowledge_msgs::DomainFormula params;
	rosplan_knowledge_msgs::DomainOperator op;
	ros::Publisher pddl_action_parameters_pub;

	/* action feedback to planning system */
	ros::Publisher action_feedback_pub;

	/* service handle to PDDL knowledge base */
	ros::ServiceClient update_knowledge_client;
	ros::ServiceClient get_knowledge_client;

	/* action status */
	rosplan_dispatch_msgs::ActionDispatch::ConstPtr current_action;

	typedef std::map<std::string, std::string> ParamObjectMap;
	ParamObjectMap boundParameters;
	std::string log_prefix_;
	ros::Duration execute_timeout_;

public:
	typedef rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request UpdateRequest;

	/* main loop for action interface */
	void runActionInterface();

	/* listen to and process action_dispatch topic */
	void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

	void initialize(const std::string& log_prefix = "[Action] ");

	bool isAcceptable(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	void sendStartEffects();
	void sendEndEffects();

	void publishEnabled();
	void publishAchieved();
	void publishFailed();

	bool lookupNumericalValue(rosplan_knowledge_msgs::KnowledgeItem& num);
	bool updateNumericalValue(rosplan_knowledge_msgs::KnowledgeItem& num);
	bool updatePredicates(std::vector<rosplan_knowledge_msgs::KnowledgeItem>& facts,
			UpdateRequest::_update_type_type operation);
	bool updateEffects(const std::vector<rosplan_knowledge_msgs::DomainFormula>& effects,
			UpdateRequest::_update_type_type operation);

        bool sendEffectADD(rosplan_knowledge_msgs::KnowledgeItem& item);
        bool sendEffectREMOVE(rosplan_knowledge_msgs::KnowledgeItem& item);

	virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) = 0;
};
}
#endif // SYNC_ACTION_INTERFACE_H_

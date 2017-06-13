#include <rosplan_interface_freiburg/async_action_interface.h>

namespace rosplan_interface_freiburg
{

/* run action interface */
void AsyncActionInterface::runActionInterface()
{

	initialize();

	// loop
	ros::Rate loopRate(1);
	ROS_INFO("KCL: (%s) Ready to receive", params.name.c_str());

	while (ros::ok())
	{

		pddl_action_parameters_pub.publish(params);

		loopRate.sleep();
		ros::spinOnce();
	}
}

/* run action interface */
void AsyncActionInterface::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	if (!isAcceptable(msg))
	{
		return;
	}

	publishEnabled();

	// call concrete implementation
	bool action_success = concreteCallback(msg);

	if (action_success)
	{
		sendStartEffects();

		sendEndEffects();

		// sleep a little
		ros::Rate big_rate(0.5);
		big_rate.sleep();

		publishAchieved();
	}
	else
	{
		publishFailed();
	}
}

void AsyncActionInterface::initialize(const std::string& log_prefix)
{
	log_prefix_ = log_prefix;
	ros::NodeHandle nh("~");

	execute_timeout_ = ros::Duration(300);

	// set action name
	nh.getParam("pddl_action_name", params.name);

	// fetch action params
	ros::service::waitForService("/kcl_rosplan/get_domain_operator_details", ros::Duration(20));
	ros::ServiceClient client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>(
			"/kcl_rosplan/get_domain_operator_details");
	rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
	srv.request.name = params.name;
	if (client.call(srv))
	{
		params = srv.response.op.formula;
		op = srv.response.op;
	}
	else
	{
		ROS_ERROR_STREAM(log_prefix_<<"could not call Knowledge Base for operator details: "<<params.name);
		return;
	}

	// collect predicates from operator description
	std::set<std::string> predicateNames;

	// effects
	std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator pit = op.at_start_add_effects.begin();
	for (; pit != op.at_start_add_effects.end(); pit++)
		predicateNames.insert(pit->name);

	pit = op.at_start_del_effects.begin();
	for (; pit != op.at_start_del_effects.end(); pit++)
		predicateNames.insert(pit->name);

	pit = op.at_end_add_effects.begin();
	for (; pit != op.at_end_add_effects.end(); pit++)
		predicateNames.insert(pit->name);

	pit = op.at_end_del_effects.begin();
	for (; pit != op.at_end_del_effects.end(); pit++)
		predicateNames.insert(pit->name);

	// simple conditions
	pit = op.at_start_simple_condition.begin();
	for (; pit != op.at_start_simple_condition.end(); pit++)
		predicateNames.insert(pit->name);

	pit = op.over_all_simple_condition.begin();
	for (; pit != op.over_all_simple_condition.end(); pit++)
		predicateNames.insert(pit->name);

	pit = op.at_end_simple_condition.begin();
	for (; pit != op.at_end_simple_condition.end(); pit++)
		predicateNames.insert(pit->name);

	// negative conditions
	pit = op.at_start_neg_condition.begin();
	for (; pit != op.at_start_neg_condition.end(); pit++)
		predicateNames.insert(pit->name);

	pit = op.over_all_neg_condition.begin();
	for (; pit != op.over_all_neg_condition.end(); pit++)
		predicateNames.insert(pit->name);

	pit = op.at_end_neg_condition.begin();
	for (; pit != op.at_end_neg_condition.end(); pit++)
		predicateNames.insert(pit->name);

	// fetch and store predicate details
	ros::service::waitForService("/kcl_rosplan/get_domain_predicate_details", ros::Duration(20));
	ros::ServiceClient predClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainPredicateDetailsService>(
			"/kcl_rosplan/get_domain_predicate_details");
	std::set<std::string>::iterator nit = predicateNames.begin();
	for (const auto& name : predicateNames)
	{
		if (name == "=")
		{
			continue;
		}
		if (predicates.find(name) != predicates.end())
			continue;
		rosplan_knowledge_msgs::GetDomainPredicateDetailsService predSrv;
		predSrv.request.name = name;
		if (predClient.call(predSrv))
		{
			predicates.insert(
					std::pair<std::string, rosplan_knowledge_msgs::DomainFormula>(name, predSrv.response.predicate));
		}
		else
		{
			ROS_ERROR_STREAM(log_prefix_<<"could not call Knowledge Base for predicate details: "<<name);
			return;
		}
	}

	// create PDDL info publisher
	pddl_action_parameters_pub = nh.advertise<rosplan_knowledge_msgs::DomainFormula>(
			"/kcl_rosplan/pddl_action_parameters", 10, true);

	// create the action feedback publisher
	action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

	// knowledge interface
	update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>(
			"/kcl_rosplan/update_knowledge_base_array");
	get_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(
			"/kcl_rosplan/get_current_knowledge");
}

bool AsyncActionInterface::isAcceptable(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	// check action name
	if (msg->name.compare(params.name) != 0)
	{
		return false;
	}
	ROS_INFO_STREAM("action recieved: "<< params.name);

	// check PDDL parameters
	std::vector<bool> found(params.typed_parameters.size(), false);
	boundParameters.clear();
	for (size_t j = 0; j < params.typed_parameters.size(); j++)
	{
		for (const auto& param: msg->parameters)
		{
			if (params.typed_parameters[j].key == param.key)
			{
				boundParameters[param.key] = param.value;
				found[j] = true;
				break;
			}
		}
		if (!found[j])
		{
			ROS_INFO_STREAM(log_prefix_<<"aborting action dispatch; malformed parameters, missing "<<params.typed_parameters[j].key);
			return false;
		}
	}
	current_action = msg;
	return true;
}

void AsyncActionInterface::sendStartEffects()
{
	//ROS_INFO_STREAM(log_prefix_<<"sending at start DELETE");
	updateEffects(op.at_start_del_effects, UpdateRequest::REMOVE_KNOWLEDGE);
	//ROS_INFO_STREAM(log_prefix_<<"sending at start ADD");
	updateEffects(op.at_start_add_effects, UpdateRequest::ADD_KNOWLEDGE);
}

void AsyncActionInterface::sendEndEffects()
{
	//ROS_INFO_STREAM(log_prefix_<<"sending at end DELETE");
	updateEffects(op.at_end_del_effects, UpdateRequest::REMOVE_KNOWLEDGE);
	//ROS_INFO_STREAM(log_prefix_<<"sending at end ADD");
	updateEffects(op.at_end_add_effects, UpdateRequest::ADD_KNOWLEDGE);
}

void AsyncActionInterface::publishEnabled()
{
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = current_action->action_id;
	fb.status = "action enabled";
	action_feedback_pub.publish(fb);
}

void AsyncActionInterface::publishAchieved()
{
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = current_action->action_id;
	fb.status = "action achieved";
	action_feedback_pub.publish(fb);
}

void AsyncActionInterface::publishFailed()
{
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = current_action->action_id;
	fb.status = "action failed";
	action_feedback_pub.publish(fb);
}

bool AsyncActionInterface::lookupNumericalValue(rosplan_knowledge_msgs::KnowledgeItem& num)
{
	rosplan_knowledge_msgs::GetAttributeService srv;
	srv.request.predicate_name = num.attribute_name;
	if (!get_knowledge_client.call(srv))
	{
		ROS_ERROR_STREAM("could not lookup numerical value "<<num.attribute_name);
		return false;
	}
	for (const auto& item : srv.response.attributes)
	{
		// same function
		if (item.attribute_name == num.attribute_name)
		{
			// same argument count
			if (item.values.size() == num.values.size())
			{
				// same arguments
				bool equal = true;
				for (size_t i = 0; i < num.values.size(); i++)
				{
					equal &= item.values[i].key == num.values[i].key;
					equal &= item.values[i].value == num.values[i].value;
				}
				if (equal)
				{
					num.function_value = item.function_value;
					return true;
				}
			}
		}
	}
	num.function_value = 0;
	return true;
}

bool AsyncActionInterface::updateNumericalValue(rosplan_knowledge_msgs::KnowledgeItem& num)
{
	rosplan_knowledge_msgs::KnowledgeUpdateServiceArray srv;
	srv.request.update_type = srv.request.ADD_KNOWLEDGE;
	srv.request.knowledge.push_back(num);
	if (!update_knowledge_client.call(srv))
	{
		ROS_ERROR_STREAM("could not update numerical value "<<num.attribute_name);
		return false;
	}
	return true;
}

bool AsyncActionInterface::updateEffects(const std::vector<rosplan_knowledge_msgs::DomainFormula>& effects,
		UpdateRequest::_update_type_type operation)
{
	rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updatePredSrv;
	rosplan_knowledge_msgs::KnowledgeItem item;
	item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
	updatePredSrv.request.update_type = operation;
	for (const auto& effect : effects)
	{
		item.attribute_name = effect.name;
		item.values.clear();
		diagnostic_msgs::KeyValue pair;
		const auto& predicate = predicates[effect.name];
//		predicate.
		for (size_t i = 0; i < predicate.typed_parameters.size(); i++)
		{
			// key from domain predicate, NOT from action params
			const auto& param_type = predicate.typed_parameters[i];
			const auto& param = effect.typed_parameters[i];
			pair.key = param_type.key;
			pair.value = boundParameters[param.key];
			item.values.push_back(pair);
		}
		updatePredSrv.request.knowledge.push_back(item);
	}
	if (! updatePredSrv.request.knowledge.empty())
	{
		if (!update_knowledge_client.call(updatePredSrv))
		{
			ROS_INFO_STREAM(log_prefix_<<"failed to update PDDL model in knowledge base.");
			return false;
		}
		//ROS_INFO_STREAM(log_prefix_<<"updated: "<<updatePredSrv.request);
	}
	return true;
}

} // close namespace

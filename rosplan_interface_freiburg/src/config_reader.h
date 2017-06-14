/*
 * config_reder.h
 *
 *  Created on: Jun 12, 2017
 *      Author: robosim
 */

#ifndef SRC_CONFIG_READER_H_
#define SRC_CONFIG_READER_H_

#include <ros/ros.h>

namespace config
{
typedef std::vector<std::string> ArgumentList;
struct NumericalFluent
{
	ArgumentList args;
	double value;
};
typedef rosplan_knowledge_msgs::KnowledgeUpdateServiceArray Update;

typedef std::list<ArgumentList> PredicateList;
typedef std::map<std::string, PredicateList> PredicateMap;

typedef std::list<NumericalFluent> NumericalFluentList;
typedef std::map<std::string, NumericalFluentList> NumericalFluentMap;

typedef std::list<std::string> ObjectInstanceList;
typedef std::map<std::string, ObjectInstanceList> ObjectMap;

void read_objects(ros::NodeHandle &n, std::string key, ObjectMap &elements)
{
	XmlRpc::XmlRpcValue value;
	n.getParam(key, value);
	if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
	{
		ROS_ERROR("[Config] Invalid configuration, %s not a map", key.c_str());
		throw std::runtime_error("Invalid configuration, objects not a map");
	}

	ROS_INFO("[Config] Objects");
	for (auto &e : value)
	{
		ObjectInstanceList instance_list;
		if (e.second.getType() == XmlRpc::XmlRpcValue::TypeString)
		{
			ROS_INFO("[Config]   %s - %s", static_cast<std::string>(e.second).c_str(), e.first.c_str());
			instance_list.push_back(static_cast<std::string>(e.second));
		}
		else if (e.second.getType() == XmlRpc::XmlRpcValue::TypeArray)
		{
			for (int i = 0; i < e.second.size(); ++i)
			{
				ROS_INFO("[Config]   %s - %s", static_cast<std::string>(e.second[i]).c_str(), e.first.c_str());
				instance_list.push_back(static_cast<std::string>(e.second[i]));
			}
		}
		elements[e.first] = instance_list;
	}
}

std::string read_arguments(XmlRpc::XmlRpcValue element, ArgumentList &args)
{
	if (element.getType() == XmlRpc::XmlRpcValue::TypeString)
	{
		std::string s = static_cast<std::string>(element);
		args.push_back(s);
		return s;
	}
	else if (element.getType() == XmlRpc::XmlRpcValue::TypeArray)
	{
		std::string s;
		for (int j = 0; j < element.size(); ++j)
		{
			s += " " + static_cast<std::string>(element[j]);
			args.push_back(static_cast<std::string>(element[j]));
		}
		return s;
	}
	return "";
}

void read_predicates(ros::NodeHandle &n, std::string key, PredicateMap &elements)
{
	XmlRpc::XmlRpcValue value;
	n.getParam(key, value);
	if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
	{
		ROS_ERROR("[Config] Invalid configuration, %s not a map", key.c_str());
		throw std::runtime_error("Invalid configuration");
	}
	ROS_INFO("[Config] Reading %s predicates", key.c_str());
	for (auto &e : value)
	{
		if (e.second.getType() != XmlRpc::XmlRpcValue::TypeArray)
		{
			ROS_ERROR("[Config] Invalid configuration, %s predicate %s not an array", key.c_str(), e.first.c_str());
			throw std::runtime_error("Invalid configuration, invalid predicate, see log");
		}
		PredicateList predicate_list;
		for (int i = 0; i < e.second.size(); ++i)
		{
			ArgumentList predarg_list;
			std::string arg_string = read_arguments(e.second[i], predarg_list);
			ROS_INFO("[Config]   (%s%s)", e.first.c_str(), arg_string.c_str());
			predicate_list.push_back(predarg_list);
		}
		elements[e.first] = predicate_list;
	}
}

void read_numericals(ros::NodeHandle &n, std::string key, NumericalFluentMap &elements)
{
	XmlRpc::XmlRpcValue value;
	n.getParam(key, value);
	if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
	{
		ROS_ERROR("[Config] Invalid configuration, %s not a map", key.c_str());
		throw std::runtime_error("Invalid configuration");
	}
	ROS_INFO("[Config] Reading %s numerical fluents", key.c_str());
	for (auto &e : value)
	{
		if (e.second.getType() != XmlRpc::XmlRpcValue::TypeArray)
		{
			ROS_ERROR("[Config] Invalid configuration, %s numerical fluents %s not an array", key.c_str(),
					e.first.c_str());
			throw std::runtime_error("Invalid configuration, invalid numerical fluents, see log");
		}
		NumericalFluentList numericals_list;
		for (int i = 0; i < e.second.size(); ++i)
		{
			XmlRpc::XmlRpcValue& nf = e.second[i];
			if (nf.getType() != XmlRpc::XmlRpcValue::TypeStruct)
			{
				ROS_ERROR("[Config] Invalid configuration, %s numerical fluent entry %s not a struct", key.c_str(),
						e.first.c_str());
				throw std::runtime_error("Invalid configuration, invalid numerical fluents, see log");
			}
			if (!nf.hasMember("args") || !nf.hasMember("value"))
			{
				ROS_ERROR("[Config] Invalid configuration, %s numerical fluent entry %s is missing args or value",
						key.c_str(), e.first.c_str());
				throw std::runtime_error("Invalid configuration, invalid numerical fluent struct, see log");
			}
			NumericalFluent fluent;
			if (nf["value"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
			{
				fluent.value = static_cast<double>(nf["value"]);
			}
			else if (nf["value"].getType() == XmlRpc::XmlRpcValue::TypeInt)
			{
				fluent.value = static_cast<int>(nf["value"]);
			}
			else
			{
				ROS_ERROR("[Config] Invalid configuration, %s numerical fluent %s value is not double or int", key.c_str(),
						e.first.c_str());
				throw std::runtime_error("Invalid configuration, invalid numerical fluent value, see log");
			}
			std::string arg_string = read_arguments(nf["args"], fluent.args);
			ROS_INFO("[Config]   (= (%s%s) %0.2f)", e.first.c_str(), arg_string.c_str(), fluent.value);
			numericals_list.push_back(fluent);
		}
		elements[e.first] = numericals_list;
	}
}

//void fill_update_srv(const PredicateMap &pmap, Update::Request& request)
//{
//	for (const auto &p : pmap)
//	{
//		// first: predicate type name, second: PredicateInstanceList
//		for (const config::ArgumentList &i : p.second)
//		{
//			// each i: ArgumentList
//			if (i.size() != predicates_[p.first].typed_parameters.size())
//			{
//				std::string param_str;
//				std::for_each(i.begin(), i.end(), [&param_str](const auto &s)
//				{	param_str += " '" + s + "'";});
//				ROS_ERROR("[Config] Cannot add predicate instance (%s%s), "
//						"got %zu parameters, expected %zu", p.first.c_str(), param_str.c_str(), i.size(),
//						predicates_[p.first].typed_parameters.size());
//				continue;
//			}
//			rosplan_knowledge_msgs::KnowledgeItem item;
//			item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
//			item.attribute_name = p.first;
//			for (int j = 0; j < i.size(); ++j)
//			{
//				diagnostic_msgs::KeyValue pair;
//				pair.key = predicates_[p.first].typed_parameters[j].key;
//				pair.value = i[j];
//				item.values.push_back(pair);
//			}
//			request.knowledge.push_back(item);
//		}
//	}
//}

}

#endif /* SRC_CONFIG_READER_H_ */

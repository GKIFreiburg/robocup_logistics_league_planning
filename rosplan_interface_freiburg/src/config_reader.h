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
		ROS_ERROR("[RP-IniSit] Invalid configuration, %s not a map", key.c_str());
		throw std::runtime_error("Invalid configuration, objects not a map");
	}

	ROS_INFO("[RP-IniSit] Objects");
	for (auto &e : value)
	{
		ObjectInstanceList instance_list;
		if (e.second.getType() == XmlRpc::XmlRpcValue::TypeString)
		{
			ROS_INFO("[RP-IniSit]   %s - %s", static_cast<std::string>(e.second).c_str(), e.first.c_str());
			instance_list.push_back(static_cast<std::string>(e.second));
		}
		else if (e.second.getType() == XmlRpc::XmlRpcValue::TypeArray)
		{
			for (int i = 0; i < e.second.size(); ++i)
			{
				ROS_INFO("[RP-IniSit]   %s - %s", static_cast<std::string>(e.second[i]).c_str(), e.first.c_str());
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
		ROS_ERROR("[RP-IniSit] Invalid configuration, %s not a map", key.c_str());
		throw std::runtime_error("Invalid configuration");
	}
	ROS_INFO("[RP-IniSit] Reading %s predicates", key.c_str());
	for (auto &e : value)
	{
		if (e.second.getType() != XmlRpc::XmlRpcValue::TypeArray)
		{
			ROS_ERROR("[RP-IniSit] Invalid configuration, %s predicate %s not an array", key.c_str(), e.first.c_str());
			throw std::runtime_error("Invalid configuration, invalid predicate, see log");
		}
		PredicateList predicate_list;
		for (int i = 0; i < e.second.size(); ++i)
		{
			ArgumentList predarg_list;
			std::string arg_string = read_arguments(e.second[i], predarg_list);
			ROS_INFO("[RP-IniSit]   (%s%s)", e.first.c_str(), arg_string.c_str());
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
		ROS_ERROR("[RP-IniSit] Invalid configuration, %s not a map", key.c_str());
		throw std::runtime_error("Invalid configuration");
	}
	ROS_INFO("[RP-IniSit] Reading %s numerical fluents", key.c_str());
	for (auto &e : value)
	{
		if (e.second.getType() != XmlRpc::XmlRpcValue::TypeArray)
		{
			ROS_ERROR("[RP-IniSit] Invalid configuration, %s numerical fluents %s not an array", key.c_str(),
					e.first.c_str());
			throw std::runtime_error("Invalid configuration, invalid numerical fluents, see log");
		}
		NumericalFluentList numericals_list;
		for (int i = 0; i < e.second.size(); ++i)
		{
			XmlRpc::XmlRpcValue& nf = e.second[i];
			if (nf.getType() != XmlRpc::XmlRpcValue::TypeStruct)
			{
				ROS_ERROR("[RP-IniSit] Invalid configuration, %s numerical fluent entry %s not a struct", key.c_str(),
						e.first.c_str());
				throw std::runtime_error("Invalid configuration, invalid numerical fluents, see log");
			}
			if (!nf.hasMember("args") || !nf.hasMember("value"))
			{
				ROS_ERROR("[RP-IniSit] Invalid configuration, %s numerical fluent entry %s is missing args or value",
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
				ROS_ERROR("[RP-IniSit] Invalid configuration, %s numerical fluent %s value is not double or int", key.c_str(),
						e.first.c_str());
				throw std::runtime_error("Invalid configuration, invalid numerical fluent value, see log");
			}
			std::string arg_string = read_arguments(nf["args"], fluent.args);
			ROS_INFO("[RP-IniSit]   (= (%s%s) %0.2f)", e.first.c_str(), arg_string.c_str(), fluent.value);
			numericals_list.push_back(fluent);
		}
		elements[e.first] = numericals_list;
	}
}

}

#endif /* SRC_CONFIG_READER_H_ */

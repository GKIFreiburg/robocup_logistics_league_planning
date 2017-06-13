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

#include <set>

#include "config_reader.h"
#include "product_decider.h"

#define GET_CONFIG(privn, n, path, var)	  \
	if (! privn.getParam(path, var)) {      \
		if (! n.getParam(path, var))					\
			ROS_ERROR_STREAM(log_prefix_<<"can not read param "<<path);  \
	}

typedef rosplan_knowledge_msgs::KnowledgeItem Item;
typedef rosplan_knowledge_msgs::GetInstanceService Instances;
typedef rosplan_knowledge_msgs::GetAttributeService Knowledge;
typedef rosplan_knowledge_msgs::KnowledgeUpdateServiceArray Update;

std::string toString(const Item& item)
{
	if (item.knowledge_type == item.INSTANCE)
	{
		return std::string(item.instance_name+" - "+item.instance_type);
	}
	std::string param_str;
	for (size_t i = 0; i < item.values.size(); ++i) {
		param_str += " " + item.values[i].key + "=" + item.values[i].value;
	}
	std::string fact("("+item.attribute_name+param_str+")");
	if (item.knowledge_type == item.FUNCTION)
	{
		return std::string("(= "+fact+" "+std::to_string(item.function_value)+")");
	}
	return fact;
}

struct ItemComparator
{
	bool operator()(const Item& i1, const Item& i2)
	{
		return toString(i1) < toString(i2);
	}
};

typedef std::set<Item, ItemComparator> ItemSet;

class UpdaterProductionSteps
{
public:
	UpdaterProductionSteps(ros::NodeHandle &n) :
			n_(n), log_prefix_("[Products] ")
	{

		decider_ = std::make_shared<Decider>();

		create_svc_update_knowledge();
		create_svc_current_knowledge();
		create_svc_current_instances();

		ros::NodeHandle privn("~");

		GET_CONFIG(privn, n_, "rs_ring_value_blue", cfg_rs_ring_value_blue_);
		GET_CONFIG(privn, n_, "rs_ring_value_green", cfg_rs_ring_value_green_);
		GET_CONFIG(privn, n_, "rs_ring_value_orange", cfg_rs_ring_value_orange_);
		GET_CONFIG(privn, n_, "rs_ring_value_yellow", cfg_rs_ring_value_yellow_);

		GET_CONFIG(privn, n_, "base_color_value_red", cfg_base_color_value_red_);
		GET_CONFIG(privn, n_, "base_color_value_black", cfg_base_color_value_black_);
		GET_CONFIG(privn, n_, "base_color_value_silver", cfg_base_color_value_silver_);

		GET_CONFIG(privn, n_, "cap_color_value_black", cfg_cap_color_value_black_);
		GET_CONFIG(privn, n_, "cap_color_value_grey", cfg_cap_color_value_grey_);

		GET_CONFIG(privn, n_, "robot_count", robot_count_);
		GET_CONFIG(privn, n_, "max_active_products", max_active_products_);

		GET_CONFIG(privn, n_, "trigger_replanning", trigger_replanning_);
		GET_CONFIG(privn, n_, "cancel_command", cancel_command_.data);

		GET_CONFIG(privn, n_, "gate_prefix", gate_prefix_);

		GET_CONFIG(privn, n_, "product_type", product_type_);
		GET_CONFIG(privn, n_, "step_type", step_type_);

		GET_CONFIG(privn, n_, "has_step_predicate", has_step_predicate_);
		GET_CONFIG(privn, n_, "initial_step_predicate", initial_step_predicate_);
		GET_CONFIG(privn, n_, "step_precedes_predicate", step_precedes_predicate_);
		GET_CONFIG(privn, n_, "step_at_machine_predicate", step_at_machine_predicate_);
		GET_CONFIG(privn, n_, "step_completed_predicate", step_completed_predicate_);
		GET_CONFIG(privn, n_, "material_required_function", material_required_function_);

		ring_colors_ =
		{
			{	rcll_ros_msgs::ProductColor::RING_BLUE, cfg_rs_ring_value_blue_},
			{	rcll_ros_msgs::ProductColor::RING_GREEN, cfg_rs_ring_value_green_},
			{	rcll_ros_msgs::ProductColor::RING_ORANGE, cfg_rs_ring_value_orange_},
			{	rcll_ros_msgs::ProductColor::RING_YELLOW, cfg_rs_ring_value_yellow_}};

		base_colors_ =
		{
			{	rcll_ros_msgs::ProductColor::BASE_RED, cfg_base_color_value_red_},
			{	rcll_ros_msgs::ProductColor::BASE_BLACK, cfg_base_color_value_black_},
			{	rcll_ros_msgs::ProductColor::BASE_SILVER, cfg_base_color_value_silver_}};

		cap_colors_ =
		{
			{	rcll_ros_msgs::ProductColor::CAP_BLACK, cfg_cap_color_value_black_},
			{	rcll_ros_msgs::ProductColor::CAP_GREY, cfg_cap_color_value_grey_}};

		cap_colors_machines_ =
		{
			{	rcll_ros_msgs::ProductColor::CAP_BLACK, "cs2"},
			{	rcll_ros_msgs::ProductColor::CAP_GREY, "cs1"}};

		config::read_predicates(privn, "default_goals", default_goals_);

		planning_command_pub_ = n.advertise<std_msgs::String>("/kcl_rosplan/planning_commands", 10, false);

		sub_order_info_ = n_.subscribe("order_info", 10, &UpdaterProductionSteps::order_info_cb, this);
		sub_machine_info_ = n_.subscribe("machine_info", 10, &UpdaterProductionSteps::machine_info_cb, this);
		sub_ring_info_ = n_.subscribe("ring_info", 10, &UpdaterProductionSteps::ring_info_cb, this);
	}

	void create_svc_update_knowledge()
	{
		svc_update_knowledge_ = n_.serviceClient<Update>(
				"/kcl_rosplan/update_knowledge_base_array", /* persistent */true);

		ROS_INFO_STREAM(log_prefix_<<"Waiting for ROSPlan service "<<svc_update_knowledge_.getService());
		svc_update_knowledge_.waitForExistence();
	}

	void create_svc_current_knowledge()
	{
		svc_current_knowledge_ = n_.serviceClient<Knowledge>(
				"/kcl_rosplan/get_current_knowledge", /* persistent */true);
		ROS_INFO_STREAM(log_prefix_<<"Waiting for ROSPlan service "<<svc_current_knowledge_.getService());
		svc_current_knowledge_.waitForExistence();
	}

	void create_svc_current_instances()
	{
		svc_current_instances_ = n_.serviceClient<Instances>(
				"/kcl_rosplan/get_current_instances", /* persistent */true);
		ROS_INFO_STREAM(log_prefix_<<"Waiting for ROSPlan service "<<svc_current_instances_.getService());
		svc_current_instances_.waitForExistence();
	}

	void create_svc_current_goals()
	{
		svc_current_goals_ = n_.serviceClient<Knowledge>(
				"/kcl_rosplan/get_current_goals", /* persistent */true);
		ROS_INFO_STREAM(log_prefix_<<"Waiting for ROSPlan service "<<svc_current_goals_.getService());
		svc_current_goals_.waitForExistence();
	}

	void create_product_goals(Product::ConstPtr& product, Update::Request& request, ItemSet& already_known)
	{
		for (const auto& step: product->steps)
		{
			// is this step already in the knowledge base?
			if (! std::all_of(already_known.begin(), already_known.end(), [step](const Item& goal)
							{	return goal.values.front().value != step.name;}))
			{
				continue;
			}
			// (step-completed ?s - step)
			Item item;
			item.knowledge_type = Item::FACT;
			item.attribute_name = step_completed_predicate_;
			diagnostic_msgs::KeyValue kv;
			kv.key = "s";
			kv.value = step.name;
			item.values.push_back(kv);
			request.knowledge.push_back(item);
		}
	}

	void create_product_facts(Product::ConstPtr product, Update::Request& request)
	{
		// product instance
		Item item;
		item.knowledge_type = Item::INSTANCE;
		item.instance_type = product_type_;
		item.instance_name = product->name;
		request.knowledge.push_back(item);

		diagnostic_msgs::KeyValue kv;
		for (const auto& step: product->steps)
		{
			// step instance
			item = Item();
			item.knowledge_type = Item::INSTANCE;
			item.instance_type = step_type_;
			item.instance_name = step.name;
			request.knowledge.push_back(item);

			// (has-step ?p - product ?s - step)
			item = Item();
			item.knowledge_type = Item::FACT;
			item.attribute_name = has_step_predicate_;
			kv = diagnostic_msgs::KeyValue();
			kv.key = "p";
			kv.value = product->name;
			item.values.push_back(kv);
			kv = diagnostic_msgs::KeyValue();
			kv.key = "s";
			kv.value = step.name;
			item.values.push_back(kv);
			request.knowledge.push_back(item);

			// (step-at-machine ?s - step ?m - machine)
			item = Item();
			item.knowledge_type = Item::FACT;
			item.attribute_name = step_at_machine_predicate_;
			kv = diagnostic_msgs::KeyValue();
			kv.key = "s";
			kv.value = step.name;
			item.values.push_back(kv);
			kv = diagnostic_msgs::KeyValue();
			kv.key = "m";
			kv.value = step.machine;
			item.values.push_back(kv);
			request.knowledge.push_back(item);

			// (= (material-required ?s - step) 2)
			if (step.name.find("ring") != std::string::npos)
			{
				item = Item();
				item.knowledge_type = Item::FUNCTION;
				item.attribute_name = material_required_function_;
				item.function_value = step.material;
				kv = diagnostic_msgs::KeyValue();
				kv.key = "s";
				kv.value = step.name;
				item.values.push_back(kv);
				request.knowledge.push_back(item);
			}
		}

		// (initial-step ?s - step)
		item = Item();
		item.knowledge_type = Item::FACT;
		item.attribute_name = initial_step_predicate_;
		kv = diagnostic_msgs::KeyValue();
		kv.key = "s";
		kv.value = product->steps.front().name;
		item.values.push_back(kv);
		request.knowledge.push_back(item);

		for (size_t i = 1; i < product->steps.size(); i++)
		{
			// (step-precedes ?s1 ?s2 - step)
			item = Item();
			item.knowledge_type = Item::FACT;
			item.attribute_name = step_precedes_predicate_;
			kv = diagnostic_msgs::KeyValue();
			kv.key = "s1";
			kv.value = product->steps[i-1].name;
			item.values.push_back(kv);
			kv = diagnostic_msgs::KeyValue();
			kv.key = "s2";
			kv.value = product->steps[i].name;
			item.values.push_back(kv);
			request.knowledge.push_back(item);
		}
	}

	void update_products()
	{
		if (!svc_current_instances_.isValid())
		{
			create_svc_current_instances();
		}
		if (!svc_current_goals_.isValid())
		{
			create_svc_current_goals();
		}
		if (!svc_current_knowledge_.isValid())
		{
			create_svc_current_knowledge();
		}
		if (!svc_update_knowledge_.isValid())
		{
			create_svc_update_knowledge();
		}

		// get product and step instances
		Instances instances;
		instances.request.type_name = product_type_;
		if (!svc_current_instances_.call(instances))
		{
			ROS_ERROR_STREAM(log_prefix_<<"Failed to lookup type "<<instances.request.type_name);
			return;
		}
		std::set<std::string> product_names(instances.response.instances.begin(), instances.response.instances.end());
		instances.request.type_name = step_type_;
		if (!svc_current_instances_.call(instances))
		{
			ROS_ERROR_STREAM(log_prefix_<<"Failed to lookup type "<<instances.request.type_name);
			return;
		}
		std::set<std::string> step_names(instances.response.instances.begin(), instances.response.instances.end());
		std::map<std::string, std::set<std::string> > product_steps;
		for (const auto& product: product_names)
		{
			for (const auto& step: step_names)
			{
				if (step.find(product) != std::string::npos)
				{
					product_steps[product].insert(step);
				}
			}
		}

		// get step-completed predicates
		Knowledge knowledge;
		knowledge.request.predicate_name = step_completed_predicate_;
		if (!svc_current_knowledge_.call(knowledge))
		{
			ROS_ERROR_STREAM(log_prefix_<<"Failed to lookup current "<<knowledge.request.predicate_name);
			return;
		}
		ItemSet current(knowledge.response.attributes.begin(), knowledge.response.attributes.end());

		// get step-completed goals
		knowledge.request.predicate_name = step_completed_predicate_;
		if (!svc_current_goals_.call(knowledge))
		{
			ROS_ERROR_STREAM(log_prefix_<<"Failed to lookup goals "<<knowledge.request.predicate_name);
			return;
		}
		ItemSet goals(knowledge.response.attributes.begin(), knowledge.response.attributes.end());

		ItemSet completed_goals;
		std::set_intersection(goals.begin(), goals.end(), current.begin(), current.end(),
				std::inserter(completed_goals, completed_goals.begin()), ItemComparator());

		// find completed products and incomplete goals
		std::set<std::string> completed_steps;
		for(const auto& goal: completed_goals)
		{
			completed_steps.insert(goal.values.front().value);
		}
		std::set<std::string> completed_products;
		std::set<std::string> incomplete_products;
		for (const auto& product: product_steps)
		{
			const auto& steps = product.second;
			if (std::all_of(steps.begin(), steps.end(), [completed_steps](const auto& step)
							{
								return completed_steps.find(step) != completed_steps.end();
							}))
			{
				completed_products.insert(product.first);
			}
			else
			{
				incomplete_products.insert(product.first);
			}
		}

		// remove completed products and corresponding steps and goals
		Update srv_facts;
		srv_facts.request.update_type = Update::Request::REMOVE_KNOWLEDGE;
		Update srv_goals;
		srv_goals.request.update_type = Update::Request::REMOVE_GOAL;
		for(const auto& product_name: completed_products)
		{
			ROS_INFO_STREAM(log_prefix_<<"Product "<<product_name<<" complete! Removing facts and goals");
			Item product;
			product.knowledge_type = Item::INSTANCE;
			product.instance_type = product_type_;
			product.instance_name = product_name;
			srv_facts.request.knowledge.push_back(std::move(product));
			for (const auto& step_name: product_steps[product_name])
			{
				Item step;
				step.knowledge_type = Item::INSTANCE;
				step.instance_type = step_type_;
				step.instance_name = step_name;
				srv_facts.request.knowledge.push_back(std::move(step));

				Item step_completed;
				step_completed.knowledge_type = Item::FACT;
				step_completed.attribute_name = step_completed_predicate_;
				diagnostic_msgs::KeyValue kv;
				kv.key = "s";
				kv.value = step_name;
				step_completed.values.push_back(std::move(kv));
				srv_goals.request.knowledge.push_back(std::move(step_completed));
			}
		}
		completed_products_.insert(completed_products.begin(), completed_products.end());
		if (!svc_update_knowledge_.call(srv_facts))
		{
			ROS_ERROR_STREAM(log_prefix_<<"Failed to remove completed product facts");
		}
		if (!svc_update_knowledge_.call(srv_goals))
		{
			ROS_ERROR_STREAM(log_prefix_<<"Failed to remove completed product goals");
		}

		Update new_product_facts;
		new_product_facts.request.update_type = Update::Request::ADD_KNOWLEDGE;
		int new_slots = std::min(max_active_products_, robot_count_) - incomplete_products.size();
		std::string new_names;
		if (new_slots > 0)
		{
			// chose additional product
			std::set<Product::ConstPtr> potential_products;
			for (const auto& name_product: available_products_)
			{
				if (incomplete_products.find(name_product.first) != incomplete_products.end())
				{
					continue;
				}
				if (completed_products_.find(name_product.first) != completed_products_.end())
				{
					continue;
				}
				potential_products.insert(name_product.second);
			}
			std::vector<Product::ConstPtr> chosen_products;
			decider_->choose_next_product(potential_products, new_slots, chosen_products);
			for (const auto& p: chosen_products)
			{
				incomplete_products.insert(p->name);
				create_product_facts(p, new_product_facts.request);
				new_names += p->name+" ";
			}
		}
		if (! new_product_facts.request.knowledge.empty())
		{
			if (svc_update_knowledge_.call(new_product_facts))
			{
				ROS_INFO_STREAM(log_prefix_<<"Adding new products "<<new_names);
			}
			else
			{
				ROS_ERROR_STREAM(log_prefix_<<"Failed to add new products");
			}
		}

		Update new_goals;
		new_goals.request.update_type = Update::Request::ADD_GOAL;
		for(const auto& incomplete_name: incomplete_products)
		{
			Product::ConstPtr& p = available_products_[incomplete_name];
			create_product_goals(p, new_goals.request, goals);
		}

		if (goals.empty() && new_goals.request.knowledge.empty())
		{
			// TODO: fallback goals if no product available
		}

		if (! new_goals.request.knowledge.empty())
		{
			if (svc_update_knowledge_.call(new_goals))
			{
				ROS_INFO_STREAM(log_prefix_<<"Adding new goals");
			}
			else
			{
				ROS_ERROR_STREAM(log_prefix_<<"Failed to add new goals");
			}
		}

		if (!new_goals.request.knowledge.empty() && trigger_replanning_)
		{
			// replan
			planning_command_pub_.publish(cancel_command_);
		}
	}

	std::string delivery_gate_to_name(int delivery_gate)
	{
		return gate_prefix_ + std::to_string(delivery_gate);
	}

	Product::ConstPtr generate_product(const rcll_ros_msgs::Order& order, int id)
	{
		std::shared_ptr<Product> p = std::make_shared<Product>();
		p->name = "p"+std::to_string(order.id)+std::to_string(id);
		p->expected_reward = 0;
		p->total_ring_materials = 0;

		// base step
		p->steps.push_back(Step());
		Step& base = p->steps.back();
		base.name = base_colors_[order.base_color]+"_base_"+p->name;
		base.machine = "bs";

		// 0-3 ring steps
		for (const auto& color: order.ring_colors)
		{
			p->steps.push_back(Step());
			Step& ring = p->steps.back();
			ring.name = ring_colors_[color]+"_ring_"+p->name;
			ring.machine = ring_colors_machines_[color];
			ring.material = ring_colors_materials_[color];
			p->expected_reward += ring.material * 2;	// 2 points awarded per material delivered
			// For successfully mounting a ring requiring x meterials, y points are awarded as- (x , y): (2, 20), (1, 10), (0, 5)
			p->expected_reward += ring.material == 2? 20 : ring.material == 1? 10 : 5;
			p->total_ring_materials += ring.material;
		}

		// cap step
		p->steps.push_back(Step());
		Step& cap = p->steps.back();
		cap.name = cap_colors_[order.cap_color]+"_cap_"+p->name;
		cap.machine = cap_colors_machines_[order.cap_color];
		// For mounting a cap, 10 points are awarded
		p->expected_reward += 10;

		// delivery step
		p->steps.push_back(Step());
		Step& delivery = p->steps.back();
		delivery.name = delivery_gate_to_name(order.delivery_gate)+"_delivery_"+p->name;
		delivery.machine = "ds";

		// For successfully delivering a product within the alotted delivery time, 20 points are awarded
		p->expected_reward += 20;

		// For mounting the last ring on a product of complexity x, y points are awarded as - (x, y): (3, 80), (2, 30), (1, 10), (0, 0)
		int comp = p->complexity();
		p->expected_reward += comp == 3? 80 : comp == 2? 30 : comp == 1? 10 : 0;

		p->earliest_delivery = order.delivery_period_begin;
		p->latest_delivery = order.delivery_period_end;

		return std::move(p);
	}

	void machine_info_cb(const rcll_ros_msgs::MachineInfo::ConstPtr& msg)
	{
		latest_machine_info_ = msg;
		for (const auto& machine: latest_machine_info_->machines)
		{
			for (const auto& color: machine.rs_ring_colors)
			{
				// C-RS1 -> rs1
				std::string name = machine.name.substr(machine.name.find("-")+1);
				std::transform(name.begin(), name.end(), name.begin(), ::tolower);
				ring_colors_machines_[color] = name;
			}
		}
	}

	void ring_info_cb(const rcll_ros_msgs::RingInfo::ConstPtr& msg)
	{
		latest_ring_info_ = msg;
		for (const auto& ring: latest_ring_info_->rings)
		{
			ring_colors_materials_[ring.ring_color] = ring.raw_material;
		}
	}

	void order_info_cb(const rcll_ros_msgs::OrderInfo::ConstPtr& msg)
	{
		latest_order_info_ = msg;
		if (! latest_machine_info_ || ! latest_ring_info_)
		{
			ROS_INFO_STREAM(log_prefix_<<"Machine info or ring info not yet available");
			return;
		}
		for(const auto& order: latest_order_info_->orders)
		{
			for (int id = 0; id < order.quantity_requested; id++)
			{
				Product::ConstPtr p = generate_product(order, id);
				available_products_[p->name] = p;
			}
		}
		update_products();
	}

	std::set<Product> filter_product_ring_test(const std::set<Product>& received_products, int max_set_size = 1)
	{
		std::set<Product> accepted_products;
		for (const auto& prod: received_products)
		{
			if (prod.complexity() == 0 || prod.complexity() == 3)
			{
				continue;
			}
			if (accepted_products.size() > max_set_size)
			{
				break;
			}
			accepted_products.insert(prod);
		}
		return accepted_products;
	}

	// Original naive filter, takes as input maximum product complexity and maximum number of products, and filters accordingly
	std::set<Product> filter_product_set_1(const std::set<Product>& received_products, const int max_complexity = 3, const int max_set_size = 1)
	{
		std::set<Product> accepted_products;
		for (const auto& prod: received_products)
		{
			if (prod.complexity() >= max_complexity)
			{
				continue;
			}
			if (accepted_products.size() > max_set_size)
			{
				continue;
			}
			accepted_products.insert(prod);
		}
		return accepted_products;
	}

	//Slight improvement over filter #1, combines complexity and number of products into a weighted 'load' value - the weights need to be tweaked
	std::set<Product> filter_product_set_2(const std::set<Product>& received_products, const int max_load = 3, const std::array<int, 4>& weights = {1, 2, 3, 4})
	{
		std::set<Product> accepted_products;
		int tmp;
		int current_load = 0;
		for (const auto& prod: received_products)
		{
			tmp = prod.complexity();
			if (weights[tmp] + current_load > max_load)
			{
				continue;
			}
			accepted_products.insert(prod);
			current_load += weights[tmp];
		}
		return accepted_products;
	}

//Slightly different filter, attempts to read off expected reward values for each product order and only allows the best product from each complexity class to be produced
	std::set<Product> filter_product_set_2(const std::set<Product>& received_products)
	{
		std::set<Product> accepted_products;
		std::array<Product, 4> best_product_for_complexity;
		std::array<int, 4> best_reward_for_complexity = {0, 0, 0, 0};
		int tmp;
		for (const auto& prod: received_products)
		{
			tmp = prod.complexity();
			if (best_reward_for_complexity[tmp] >= prod.expected_reward)
			{
				continue;
			}
			best_product_for_complexity[tmp] = prod;
		}

		for (const auto& prod: best_product_for_complexity)
		{
			accepted_products.insert(prod);
		}

		return accepted_products;
	}

private:
	ros::NodeHandle n_;

	ros::Publisher planning_command_pub_;
	ros::Subscriber sub_order_info_;
	ros::Subscriber sub_machine_info_;
	ros::Subscriber sub_ring_info_;
	ros::ServiceClient svc_update_knowledge_;
	ros::ServiceClient svc_current_knowledge_;
	ros::ServiceClient svc_current_instances_;
	ros::ServiceClient svc_current_goals_;

	int robot_count_;
	int max_active_products_;

	std::string product_type_;
	std::string step_type_;

	std::string has_step_predicate_;
	std::string initial_step_predicate_;
	std::string step_precedes_predicate_;
	std::string step_at_machine_predicate_;
	std::string step_completed_predicate_;
	std::string material_required_function_;

	std::string cfg_rs_ring_value_blue_;
	std::string cfg_rs_ring_value_green_;
	std::string cfg_rs_ring_value_orange_;
	std::string cfg_rs_ring_value_yellow_;

	std::string cfg_base_color_value_red_;
	std::string cfg_base_color_value_black_;
	std::string cfg_base_color_value_silver_;

	std::string cfg_cap_color_value_black_;
	std::string cfg_cap_color_value_grey_;

	std::string gate_prefix_;

	bool trigger_replanning_;
	std_msgs::String cancel_command_;
	config::PredicateMap default_goals_;

	std::shared_ptr<Decider> decider_;

	std::map<int, std::string> ring_colors_;
	std::map<int, std::string> base_colors_;
	std::map<int, std::string> cap_colors_;

	std::map<int, int> ring_colors_materials_;
	std::map<int, std::string> ring_colors_machines_;
	std::map<int, std::string> cap_colors_machines_;

	rcll_ros_msgs::OrderInfo::ConstPtr latest_order_info_;
	rcll_ros_msgs::MachineInfo::ConstPtr latest_machine_info_;
	rcll_ros_msgs::RingInfo::ConstPtr latest_ring_info_;

	std::map<std::string, Product::ConstPtr> available_products_;
	std::set<std::string> current_products_;
	std::set<std::string> completed_products_;
	std::set<std::string> incomplete_products_;

	std::string log_prefix_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "updater_production_steps");

	ros::NodeHandle n;

	UpdaterProductionSteps product_updater(n);

	ros::spin();

	return 0;
}

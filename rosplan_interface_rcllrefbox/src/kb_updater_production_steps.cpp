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

#define GET_CONFIG(privn, n, path, var)	  \
	if (! privn.getParam(path, var)) {      \
		n.getParam(path, var);                \
	}

struct Step
{
	std::string name;
	std::string machine;
	int material;
};

struct Product
{
	std::string name;
	std::vector<Step> steps;
	size_t complexity() const
	{
		return steps.size() - 3;
	}
	bool operator<(const Product& other) const
	{
		return name < other.name;
	}
};

typedef rosplan_knowledge_msgs::KnowledgeUpdateServiceArray Update;
typedef rosplan_knowledge_msgs::KnowledgeItem Item;

class ROSPlanKbUpdaterProductionSteps
{
public:
	ROSPlanKbUpdaterProductionSteps(ros::NodeHandle &n) :
			n(n)
	{
		sub_order_info_ = n.subscribe("rcll/order_info", 10, &ROSPlanKbUpdaterProductionSteps::order_info_cb, this);
		sub_machine_info_ = n.subscribe("rcll/machine_info", 10, &ROSPlanKbUpdaterProductionSteps::machine_info_cb, this);
		sub_ring_info_ = n.subscribe("rcll/ring_info", 10, &ROSPlanKbUpdaterProductionSteps::ring_info_cb, this);

		create_svc_update_knowledge();
		create_svc_current_knowledge();
		create_svc_current_instances();

		ros::NodeHandle privn("~");

		GET_CONFIG(privn, n, "rs_ring_value_blue", cfg_rs_ring_value_blue_);
		GET_CONFIG(privn, n, "rs_ring_value_green", cfg_rs_ring_value_green_);
		GET_CONFIG(privn, n, "rs_ring_value_orange", cfg_rs_ring_value_orange_);
		GET_CONFIG(privn, n, "rs_ring_value_yellow", cfg_rs_ring_value_yellow_);

		GET_CONFIG(privn, n, "base_color_value_red", cfg_base_color_value_red_);
		GET_CONFIG(privn, n, "base_color_value_black", cfg_base_color_value_black_);
		GET_CONFIG(privn, n, "base_color_value_silver", cfg_base_color_value_silver_);

		GET_CONFIG(privn, n, "cap_color_value_black", cfg_cap_color_value_black_);
		GET_CONFIG(privn, n, "cap_color_value_grey", cfg_cap_color_value_grey_);

		GET_CONFIG(privn, n, "gate_prefix", gate_prefix_);

		GET_CONFIG(privn, n, "product_type", product_type_);
		GET_CONFIG(privn, n, "step_type", step_type_);

		GET_CONFIG(privn, n, "has_step_predicate", has_step_predicate_);
		GET_CONFIG(privn, n, "initial_step_predicate", initial_step_predicate_);
		GET_CONFIG(privn, n, "step_precedes_predicate", step_precedes_predicate_);
		GET_CONFIG(privn, n, "step_at_machine_predicate", step_at_machine_predicate_);
		GET_CONFIG(privn, n, "step_completed_predicate", step_completed_predicate_);
		GET_CONFIG(privn, n, "material_required_function", material_required_function_);

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
	}

	void create_svc_update_knowledge()
	{
		svc_update_knowledge_ = n.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>(
				"kcl_rosplan/update_knowledge_base_array", /* persistent */true);

		ROS_INFO("[KBU-ProdStep] Waiting for ROSPlan service update_knowledge_base");
		svc_update_knowledge_.waitForExistence();
	}

	void create_svc_current_knowledge()
	{
		svc_current_knowledge_ = n.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(
				"kcl_rosplan/get_current_knowledge", /* persistent */true);
		ROS_INFO("[KBU-ProdStep] Waiting for ROSPlan service get_current_knowledge");
		svc_current_knowledge_.waitForExistence();
	}

	void create_svc_current_instances()
	{
		svc_current_instances_ = n.serviceClient<rosplan_knowledge_msgs::GetInstanceService>(
				"kcl_rosplan/get_current_instances", /* persistent */true);
		ROS_INFO("[KBU-ProdStep] Waiting for ROSPlan service get_current_instances");
		svc_current_instances_.waitForExistence();
	}

	void create_product_goals(const Product& product,
			rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest& request)
	{
		for (const auto& step: product.steps)
		{
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

	void create_product_facts(const Product& product,
			rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest& request)
	{
		// product instance
		Item item;
		item.knowledge_type = Item::INSTANCE;
		item.instance_type = product_type_;
		item.instance_name = product.name;
		request.knowledge.push_back(item);

		diagnostic_msgs::KeyValue kv;
		for (const auto& step: product.steps)
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
			kv.value = product.name;
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
		kv.value = product.steps.front().name;
		item.values.push_back(kv);
		request.knowledge.push_back(item);

		for (size_t i = 1; i < product.steps.size(); i++)
		{
			// (step-precedes ?s1 ?s2 - step)
			item = Item();
			item.knowledge_type = Item::FACT;
			item.attribute_name = step_precedes_predicate_;
			kv = diagnostic_msgs::KeyValue();
			kv.key = "s1";
			kv.value = product.steps[i-1].name;
			item.values.push_back(kv);
			kv = diagnostic_msgs::KeyValue();
			kv.key = "s2";
			kv.value = product.steps[i].name;
			item.values.push_back(kv);
			request.knowledge.push_back(item);
		}
	}

	void publish_product_knowledge(std::vector<Product>& products, Update::Request::_update_type_type op)
	{

		Update facts;
		Update goals;
		std::string verb;
		switch (op)
		{
			case Update::Request::ADD_KNOWLEDGE:
			case Update::Request::ADD_GOAL:
			facts.request.update_type = Update::Request::ADD_KNOWLEDGE;
			goals.request.update_type = Update::Request::ADD_GOAL;
			verb = "add";
			break;
			case Update::Request::REMOVE_KNOWLEDGE:
			case Update::Request::REMOVE_GOAL:
			facts.request.update_type = Update::Request::REMOVE_KNOWLEDGE;
			goals.request.update_type = Update::Request::REMOVE_GOAL;
			verb = "remove";
			break;
		}
		for (const auto&product: products)
		{
			create_product_facts(product, facts.request);
			create_product_goals(product, goals.request);
		}
		if (!svc_update_knowledge_.isValid())
		{
			create_svc_update_knowledge();
		}
		if (!svc_update_knowledge_.call(facts))
		{
			ROS_ERROR_STREAM("[KBU-ProdStep] Failed to "<<verb<<" facts");
			return;
		}
		if (!svc_update_knowledge_.call(goals))
		{
			ROS_ERROR_STREAM("[KBU-ProdStep] Failed to "<<verb<<" goals");
			return;
		}
	}

	void update_products(std::set<Product>& accepted)
	{
		std::vector<Product> add_products;
		std::set_difference(accepted.begin(), accepted.end(), current_products_.begin(), current_products_.end(),
				std::back_inserter(add_products));
		std::vector<Product> remove_products;
		std::set_difference(current_products_.begin(), current_products_.end(), accepted.begin(), accepted.end(),
				std::back_inserter(remove_products));
		if (remove_products.empty() && add_products.empty())
		{
			return;
		}
		if (! remove_products.empty())
		{
			ROS_INFO_STREAM("[KBU-ProdStep] Removing "<<remove_products.size()<<" products");
			publish_product_knowledge(remove_products, Update::Request::REMOVE_KNOWLEDGE);
		}
		if (! add_products.empty())
		{
			ROS_INFO_STREAM("[KBU-ProdStep] Adding "<<add_products.size()<<" products");
			publish_product_knowledge(add_products, Update::Request::ADD_KNOWLEDGE);
		}
		current_products_ = accepted;
	}

	std::string delivery_gate_to_name(int delivery_gate)
	{
		return gate_prefix_ + std::to_string(delivery_gate);
	}

	Product generate_product(const rcll_ros_msgs::Order& order, int id)
	{
		Product p;
		p.name = "p"+std::to_string(order.id)+std::to_string(id);

		// base step
		p.steps.push_back(Step());
		Step& base = p.steps.back();
		base.name = base_colors_[order.base_color]+"_base_"+p.name;
		base.machine = "bs";

		// 0-3 ring steps
		for (const auto& color: order.ring_colors)
		{
			p.steps.push_back(Step());
			Step& ring = p.steps.back();
			ring.name = ring_colors_[color]+"_ring_"+p.name;
			ring.machine = ring_colors_machines_[color];
			ring.material = ring_colors_materials_[color];
		}

		// cap step
		p.steps.push_back(Step());
		Step& cap = p.steps.back();
		cap.name = cap_colors_[order.cap_color]+"_cap_"+p.name;
		cap.machine = cap_colors_machines_[order.cap_color];

		// delivery step
		p.steps.push_back(Step());
		Step& delivery = p.steps.back();
		delivery.name = delivery_gate_to_name(order.delivery_gate)+"_delivery_"+p.name;
		delivery.machine = "ds";

		return p;
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
			ROS_INFO("[KBU-ProdStep] Machine info or ring info not yet available");
			return;
		}
		std::set<Product> received_products;
		for(const auto& order: latest_order_info_->orders)
		{
			for (int id = 0; id < order.quantity_requested; id++)
			{
				received_products.insert(generate_product(order, id));
			}
		}

		std::set<Product> accepted_products;
		for (const auto& p: received_products)
		{
			// TODO: filter products
//			if (p.complexity() == 0)
			{
				accepted_products.insert(p);
			}
		}

		update_products(accepted_products);
	}

private:
	ros::NodeHandle n;

	ros::Subscriber sub_order_info_;
	ros::Subscriber sub_machine_info_;
	ros::Subscriber sub_ring_info_;
	ros::ServiceClient svc_update_knowledge_;
	ros::ServiceClient svc_current_knowledge_;
	ros::ServiceClient svc_current_instances_;

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

	std::map<int, std::string> ring_colors_;
	std::map<int, std::string> base_colors_;
	std::map<int, std::string> cap_colors_;

	std::map<int, int> ring_colors_materials_;
	std::map<int, std::string> ring_colors_machines_;
	std::map<int, std::string> cap_colors_machines_;

	rcll_ros_msgs::OrderInfo::ConstPtr latest_order_info_;
	rcll_ros_msgs::MachineInfo::ConstPtr latest_machine_info_;
	rcll_ros_msgs::RingInfo::ConstPtr latest_ring_info_;

	std::set<Product> current_products_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rosplan_interface_behaviorengine");

	ros::NodeHandle n;

	ROSPlanKbUpdaterProductionSteps rosplan_kb_updater(n);

	ros::spin();

	return 0;
}

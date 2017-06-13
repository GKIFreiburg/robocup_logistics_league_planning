/*
 * product_decider.h
 *
 *  Created on: Jun 11, 2017
 *      Author: robosim
 */

#ifndef SRC_PRODUCT_DECIDER_H_
#define SRC_PRODUCT_DECIDER_H_

#include <ros/ros.h>

struct Step
{
	std::string name;
	std::string machine;
	int material;
};

struct Product
{
	typedef std::shared_ptr<const Product> ConstPtr;
	size_t complexity() const
	{
		return steps.size() - 3;
	}
	bool operator<(const Product& other) const
	{
		return name < other.name;
	}
	std::string name;
	std::vector<Step> steps;
	int total_ring_materials;
	int expected_reward;
	unsigned int earliest_delivery;
	unsigned int latest_delivery;
};

class Decider
{

public:
	Decider():
		config_("only_c0")
{
		ros::NodeHandle nh("~");
		nh.getParam("decider_config", config_);
}

	void choose_next_product(
			const std::set<Product::ConstPtr>& potential_products,
			int max_products,
			std::vector<Product::ConstPtr>& chosen_products)
	{
		ROS_INFO_STREAM("[Decider] "<<"choosing "<<max_products<<" additional products according to rule '"<<config_<<"'");
		if (config_ == "only_c0")
		{
			choose_next_c0(potential_products, max_products, chosen_products);
		}
		else if (config_ == "c1_c2")
		{
			choose_next_c1_c2(potential_products, max_products, chosen_products);
		}
		else if (config_ == "no_c0")
		{
			choose_next_no_c0(potential_products, max_products, chosen_products);
		}
		while (chosen_products.size() > max_products)
		{
			chosen_products.pop_back();
		}
	}

	void choose_next_c0(
			const std::set<Product::ConstPtr>& potential_products,
			int max_products,
			std::vector<Product::ConstPtr>& chosen_products)
	{
		for (const auto& p: potential_products)
		{
			if (p->complexity() == 0)
			{
				chosen_products.push_back(p);
				if (chosen_products.size() == max_products)
				{
					return;
				}
			}
		}
	}

	void choose_next_c1_c2(
			const std::set<Product::ConstPtr>& potential_products,
			int max_products,
			std::vector<Product::ConstPtr>& chosen_products)
	{
		for (const auto& p: potential_products)
		{
			if (p->complexity() > 0 && p->complexity() < 3)
			{
				chosen_products.push_back(p);
				if (chosen_products.size() == max_products)
				{
					return;
				}
			}
		}
	}

	void choose_next_no_c0(
			const std::set<Product::ConstPtr>& potential_products,
			int max_products,
			std::vector<Product::ConstPtr>& chosen_products)
	{
		for (const auto& p: potential_products)
		{
			if (p->complexity() > 0)
			{
				chosen_products.push_back(p);
				if (chosen_products.size() == max_products)
				{
					return;
				}
			}
		}
	}

private:
	std::string config_;
};



#endif /* SRC_PRODUCT_DECIDER_H_ */

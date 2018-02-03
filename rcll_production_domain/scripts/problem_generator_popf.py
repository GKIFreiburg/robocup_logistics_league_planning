#!/usr/bin/env python
import os
import sys
import itertools
import random
import rospy
import glob

params = {}

class Problem:
	def __init__(self, orders, robot_count):
		self.name = ''
		self.objects = []
		self.initial_facts = []
		self.goal_facts = []
		self.robot_count = 0
		self.product_steps = {} # {product: [step1, ...]}
		self.total_material_cost = 0
		self.product_complexity_count = [0, 0, 0, 0]
		
		for i in range(robot_count):
			self.add_robot()
		for complexity in orders:
			self.add_product(complexity)
		self._update_name()
	
	def add_object(self, name, type):
		self.objects.append('    {name} - {type}'.format(name=name, type=type))
		
	def add_robot(self):
		self.robot_count += 1
		name = params['robot']['name_pattern'].format(index=self.robot_count)
		self.add_object(name, params['robot']['type'])
		self.initial_facts.append(params['robot_facts'].format(name=name))

	def add_step(self, detail, operation, product, station):
		name = params['step']['name_pattern'].format(detail=detail, operation=operation, product=product)
		self.add_object(name, params['step']['type'])

		if product not in self.product_steps:
			self.product_steps[product] = [name]
			self.initial_facts.append(params['initial_step'].format(step=name))
		else:
			self.initial_facts.append(params['step_precedence'].format(prev=self.product_steps[product][-1], step=name))
		if operation == 'ring':
			cost = int(params['material_costs'][detail])
			self.total_material_cost += cost
			cost_value = params['material_cost_values'][cost]
			self.initial_facts.append(params['step_material'].format(step=name, cost=cost_value))
			
		self.initial_facts.append(params['product_step'].format(step=name, product=product))
		self.initial_facts.append(params['step_station'].format(step=name, station=station))
		self.initial_facts.append(params['step_incomplete'].format(step=name))
		
		self.goal_facts.append(params['step_goal'].format(step=name))

	def add_product(self, complexity):
		self.product_complexity_count[complexity] += 1
		name = params['product']['name_pattern'].format(index=len(self.product_steps.viewkeys())+1)
		self.add_object(name, params['product']['type'])
		
		# initial step
		self.add_step(detail=random.choice(params['base_colors']), operation='base', product=name, station='bs')
		
		# ring steps
		for ring_color in random.sample(params['ring_colors'], complexity):
			self.add_step(detail=ring_color, operation='ring', product=name, station=params['rings_at_station'][ring_color])
			
		# cap step
		cap_color = random.choice(params['cap_colors'])
		self.add_step(detail=cap_color, operation='cap', product=name, station=params['caps_at_station'][cap_color])

		# delivery step
		self.add_step(detail=random.choice(params['delivery_gates']), operation='delivery', product=name, station='ds')

	def _update_name(self):
		name_parts = []
		name_parts.append('{material:02}m'.format(material=self.total_material_cost))
		for complexity, count in enumerate(self.product_complexity_count):
			name_parts.append('{count}c{complexity}'.format(count=count, complexity=complexity))
		name_parts.append('{robot_count}r'.format(robot_count=self.robot_count))
		name_parts.append('p')
		self.name = '_'.join(reversed(name_parts))

	def __str__(self):
		template_path = params['problem_template_path']
		with open(template_path) as f:
			pddl = f.read()
			return pddl.format(name=self.name, 
				objects='\n'.join(self.objects), 
				initial_facts='\n'.join(self.initial_facts), 
				goal_facts='\n'.join(self.goal_facts))

	@classmethod
	def generate_problem(cls, orders, robot_count):
		return cls(orders, robot_count)

def generate_problem_files(pddl_path, max_robots, max_products):
	# create benchmark directory
	if not os.path.exists(pddl_path):
		os.makedirs(pddl_path)
	os.chdir(pddl_path)
	# remove old files
	for file in glob.glob('*'):
		os.remove(file)
		
	# 1 to max products of complexity 0 to 3, 1 to max robots
	for robot_count in range(1, max_robots+1):
		for product_count in range(1, max_products+1):
			for orders in itertools.combinations_with_replacement(range(4), product_count):
				problem = Problem.generate_problem(orders, robot_count)
				rospy.loginfo('writing problem {}'.format(problem.name))
				with open(os.path.join(pddl_path, problem.name+'.pddl'), 'w') as f:
					f.write(str(problem))
	
if __name__=='__main__':
	rospy.init_node('problem_generator', argv=sys.argv, anonymous=True)
	params = rospy.get_param('~')
	rospy.loginfo('generation problem instances to {}'.format(params['output_path']))
	generate_problem_files(params['output_path'], params['max_robot_count'], params['max_product_count'])
	rospy.loginfo('done')
	


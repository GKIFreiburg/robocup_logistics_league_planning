#!/usr/bin/env python
import os
import sys
import itertools
import random
import rospkg
import copy

#package = "rcll_production_domain"
#package_path = rospkg.RosPack().get_path(package)
#pddl_path = os.path.join(package_path, 'freiburg_durations')

# templates
problem_template = ''
product_object_template = """	{name} - product
	init_{name} {ring_steps} cap_{name} delivery_{name} - step

"""
ring_step_template = """ ring{index}_{name}"""
product_init_template = """	;{name}
	(has-step {name} init_{name})
	(initial-step init_{name})
	(step-at-machine init_{name} bs)
	
{rings}	(has-step {name} cap_{name})
	(step-precedes {pre_cap} cap_{name})
	(step-at-machine cap_{name} cs{cap_station})

	(has-step {name} delivery_{name})
	(step-precedes cap_{name} delivery_{name})
	(step-at-machine delivery_{name} ds)

"""
ring_init_template = """	(has-step {name} ring{index}_{name})
	(step-precedes init_{name} ring{index}_{name})
	(step-at-machine ring{index}_{name} rs{machine})
	(= (material-required ring{index}_{name}) {complexity})

"""
robot_template = """	(robot-at-init r{index})
"""

def generate_product(ring_complexities, cap_station, product_id):
	name = 'p{id}'.format(id=product_id)
	ring_steps = ''
	pre_cap_step = 'init_{name}'.format(name=name)
	rings = ''
	for index, complexity in enumerate(ring_complexities):
		pre_cap_step = ring_step_template.format(index=index+1, name=name)
		ring_steps += pre_cap_step
		machine = complexity % 2 + 1
		if complexity - 1 >= 0: 
			complexity -= 1
		rings += ring_init_template.format(name=name, index=index+1, complexity=complexity, machine=machine)
	product_object = product_object_template.format(name = name, ring_steps=ring_steps)
	product_init = product_init_template.format(name=name, rings=rings, pre_cap=pre_cap_step, cap_station=cap_station)
	return product_object, product_init

def generate_problem(orders, robot_count=3):
	name = 'p_{robot_count}r'.format(robot_count=robot_count)
	robots = ''
	for i in range(robot_count):
		robots += robot_template.format(index=i+1)
	products = ''
	product_inits = ''
	product_id = 1
	complexity_counts = [0,0,0,0]
	for complexity in orders:
		complexity_counts[int(complexity)] += 1
		rings = random.sample(range(4), int(complexity))
		cap = random.choice([1,2])
		p, i = generate_product(ring_complexities = rings, cap_station=cap, product_id=product_id)
		products += p
		product_inits += i
		product_id += 1
	for complexity, count in enumerate(complexity_counts):
		name += '_{count}c{complexity}'.format(count=count, complexity=complexity)
	problem = problem_template.format(name=name, robots=robots, products=products, product_inits=product_inits)
	return problem, name

def generate_problem_files(pddl_path):
	# 1 to 4 products of complexity 0 to 3, 1 to 3 robots
	for robot_count in range(1, 4):
		for product_count in range(1,5):
			for product_complexities in itertools.combinations_with_replacement('0123', product_count):
				problem, name = generate_problem(product_complexities, robot_count=robot_count)
				with open(os.path.join(pddl_path, name+'.pddl'), 'w') as f:
					f.write(problem)
	
if __name__=='__main__':
	if len(sys.argv) != 2:
		print('usage: problem_generator.py path/to/pddl_output/')
		exit(0)
	pddl_path = sys.argv[1]
	templates_path = os.path.join(rospkg.RosPack().get_path('rcll_production_domain'),'templates')
	problem_template_file = os.path.join(templates_path, 'problem.txt')
	with open(problem_template_file, 'r') as f:
		problem_template = f.read()
		
	generate_problem_files(pddl_path)
		
		

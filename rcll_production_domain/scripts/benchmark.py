#!/usr/bin/env python
import os
import sys
import itertools
import subprocess

#package = "rcll_production_domain"
#package_path = rospkg.RosPack().get_path(package)
#pddl_path = os.path.join(package_path, 'freiburg_durations')

cmd = "roslaunch rcll_production_domain tfd_plan.launch domain:={d} problem:={p}"
plan_times_filename = os.path.expanduser('~/.ros/plan.times')

def run_benchmark(pddl_path, benchmark_filename):
	domain_names = []
	problem_names = []
	pddl_path = os.path.abspath(pddl_path)
	for file_name in os.listdir(pddl_path):
		index = file_name.rfind('.pddl')
		if index == -1:
			continue
		# pddl file
		name = file_name[0:index]
		if name.find('d') == 0:
			domain_names.append(name)
		elif name.find('p') == 0:
			problem_names.append(name)

	print('found domains: {}'.format(str(domain_names)))
	#print('found problems: {}'.format(str(problem_names)))
	benchmark_file = os.path.join(pddl_path, benchmark_filename)
	with open(benchmark_file, 'w') as f:
		f.write('domain,robots,c0,c1,c2,c3,first makespan,first plan time,best makespan,best plan time,timeout\n')
	for domain, problem in itertools.product(domain_names, problem_names):
		print('starting benchmark for {d} and {p}...'.format(d=domain, p=problem))
		benchmark_line = run_planner(pddl_path, domain, problem)
		with open(benchmark_file, 'a') as f:
			f.write(benchmark_line)

def run_planner(pddl_path, domain_name, problem_name):
	if os.path.exists(plan_times_filename):
		os.remove(plan_times_filename)
	domain_path = os.path.join(pddl_path, domain_name+'.pddl')
	problem_path = os.path.join(pddl_path, problem_name+'.pddl')
	planner_cmd = cmd.format(d=domain_path, p=problem_path)
	print('calling planner: {}'.format(planner_cmd))
	subprocess.call(planner_cmd.split())
	print('planning exited; gathering results...')
	
	lines = []
	with open(plan_times_filename) as f:
		lines = f.readlines()
	makespan = 'INF'
	search_time = 0
	first_makespan = 'INF'
	first_search_time = 0
	wall_time = 0
	for line in lines:
		if line.find('#') == 0:
			continue
		values = line.split()
		if float(values[0]) == -1:
			wall_time = float(values[4])
		else:
			makespan = float(values[0])
			search_time = float(values[4])
			if first_makespan == 'INF':
				first_makespan = makespan
				first_search_time = search_time
	# p_3r_1c0_2c1_0c2_0c3
	problem_complexity = [int(count[0]) for count in problem_name[2:].split('_')]
	results = [domain_name]
	results.extend(problem_complexity)
	results.extend([first_makespan, first_search_time, makespan, search_time, wall_time])
	return '{},{},{},{},{},{},{},{},{},{},{}\n'.format(*results)
	
if __name__=='__main__':
	if len(sys.argv) < 2:
		print('usage: benchmark.py path/to/pddl/')
		exit(0)
	result_filename = 'benchmark.csv'
	pddl_path = sys.argv[1]
	run_benchmark(pddl_path, result_filename)
		
		

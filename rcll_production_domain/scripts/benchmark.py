#!/usr/bin/env python
import os
import sys
#import rospkg

#package = "rcll_production_domain"
#package_path = rospkg.RosPack().get_path(package)
#pddl_path = os.path.join(package_path, 'freiburg_durations')

cmd = "roslaunch rcll_production_domain tfd_plan_durations.launch dname:={domain} pname:={problem}"
result_filename = os.path.expanduser('~/.ros/plan.times')

def run_benchmark(pddl_path, benchmark_filename)
	domain_names = []
	problem_names = []
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
	print('found problems: {}'.format(str(problem_names)))
	for domain, problem in itertools.product(domain_names, problem_names):
		print('starting benchmark for {d} and {p}...'.format(d=domain, p=problem)
		benchmark_line = run_planner(domain, problem)
		with f = open(benchmark_filename, 'a'):
			f.append(benchmark_line)

def run_planner(domain, problem)
	os.remove(result_filename)
	process = Popen(cmd.format(domain=domain, problem=problem))
	process.wait()

	lines = []
	with f = open(result_filename):
		lines = f.read().split('\n')
	makespan = 0
	search_time = 0
	wall_time = 0
	for line in lines:
		if line.find('#') == 0:
			continue
		values = line.split()
		if values[0] == -1:
			wall_time = float(values[4])
		else
			makespan = float(values[0])
			search_time = float(values[4])
	return '{},{},{},{},{}\n'.format(domain, problem, makespan, search_time, wall_time)
	
if __name__=='__main__':
	if len(sys.argv) < 2:
		print('usage: benchmark.py path/to/pddl/ [result_filename]
	result_filename = 'benchmark.cvs'
	if len(sys.argv) >= 3:
		result_filename = sys.argv[2]
	pddl_path = sys.argv[1]
	run_benchmark(pddl_path, result_filename)
		
		

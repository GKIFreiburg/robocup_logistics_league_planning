#!/usr/bin/env python
import os
import sys
import itertools
import subprocess

cmd = "roslaunch rcll_production_domain tfd_plan.launch domain:={d} problem:={p}"
plan_filename = os.path.expanduser('~/.ros/plan.best')

if __name__=='__main__':
	if len(sys.argv) != 3:
		print('usage: tfd_plan.py /path/to/domain.pddl /path/to/problem.pddl')
		exit(0)
		
	domain_filename = sys.argv[1]
	problem_filename = sys.argv[2]

	if os.path.exists(plan_filename):
		os.remove(plan_filename)
	domain_path = os.path.join(pddl_path, domain_name+'.pddl')
	problem_path = os.path.join(pddl_path, problem_name+'.pddl')
	planner_cmd = cmd.format(d=domain_path, p=problem_path)
	#print('calling planner: {}'.format(planner_cmd))
	subprocess.call(planner_cmd.split())
	#print('planning exited; gathering results...')
	
	with open(plan_filename) as f:
		print(f.read())
		

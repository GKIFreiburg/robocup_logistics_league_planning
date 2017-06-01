#!/usr/bin/env python
import os
import sys
import itertools
import subprocess

cmd = "roslaunch rcll_production_domain tfd_plan.launch domain:={d} problem:={p}"
plan_filename = os.path.expanduser('~/.ros/plan.best')
dump_path = '/tmp/tfd_output'

if __name__=='__main__':
	if len(sys.argv) != 3:
		print('usage: tfd_plan.py /path/to/domain.pddl /path/to/problem.pddl')
		exit(0)
		
	domain_path = os.path.abspath(sys.argv[1])
	problem_path = os.path.abspath(sys.argv[2])

	if os.path.exists(plan_filename):
		os.remove(plan_filename)
	planner_cmd = cmd.format(d=domain_path, p=problem_path)
	#print('calling planner: {}'.format(planner_cmd))
	with open(dump_path, 'w') as dump:
		subprocess.call(planner_cmd.split(), stdout=dump, stderr=dump)
	#print('planning exited; gathering results...')
	
	with open(plan_filename) as f:
		print(f.read())
		

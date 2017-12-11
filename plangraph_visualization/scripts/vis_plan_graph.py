#!/usr/bin/env python
import sys
from jsonschema._validators import dependencies

sys.path.append('/usr/lib/pyshared/python2.6')

import argparse
import re

import rospy
import colorsys
import numpy as np

from graphviz import Source

from rosplan_dispatch_msgs.msg import CompletePlan
from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan_dispatch_msgs.msg import ActionFeedback
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import String

class Visualization:
	def __init__(self):
		self.plan = []  # ActionDispatch[]
		
		self.achieved_action_ids = set()
		self.enabled_action_ids = set()
		self.failed_action_ids = set()
		
		self.color_table = {}
		self.update_params()

	def plan_callback(self, msg):
		rospy.loginfo('plan')
		self.plan = msg.plan
		self.achieved_action_ids.clear()
		self.enabled_action_ids.clear()
		self.failed_action_ids.clear()
		self.timer = rospy.Timer(period=rospy.Duration(0.1), callback=self.schedule_update, oneshot=True)

	def action_feedback_callback(self, msg):
		if msg.action_id >= len(self.plan):
			self.achieved_action_ids.clear()
			self.enabled_action_ids.clear()
			self.failed_action_ids.clear()
			return

		if msg.status == 'action enabled':
			self.enabled_action_ids.add(msg.action_id)
		elif msg.status == 'action achieved':
			if msg.action_id in self.enabled_action_ids:
				self.enabled_action_ids.remove(msg.action_id)
			self.achieved_action_ids.add(msg.action_id)
		elif msg.status == 'action failed':
			if msg.action_id in self.enabled_action_ids:
				self.enabled_action_ids.remove(msg.action_id)
			self.failed_action_ids.add(msg.action_id)
		else:
			rospy.logwarn('unknown action feedback status: {}'.format(msg.status))
		self.timer = rospy.Timer(period=rospy.Duration(0.1), callback=self.schedule_update, oneshot=True)

	def deduce_action_dependencies(self):
		# dependency graph in matrix representation
		action_count = len(self.plan)
		dependency_matrix = np.zeros(shape=(action_count, action_count))

		for action in self.plan:
			parameter_values = set(param.value for param in action.parameters)
			# find action dependency on earlier actions
			for other_action in self.plan:
				other_end_time = other_action.dispatch_time + other_action.duration
				if action.dispatch_time + 0.0001 < other_end_time:
					continue
				other_parameter_values = set(param.value for param in other_action.parameters)
				if len(parameter_values & other_parameter_values) > 0:
					dependency_matrix[action.action_id, other_action.action_id] = 1

		# transitive reduction
		for j in range(action_count):
			for i in range(action_count):
				if dependency_matrix[i, j]:
					for k in range(action_count):
						if dependency_matrix[j, k]:
							dependency_matrix[i, k] = 0

		self.dependency_matrix = dependency_matrix

	def schedule_update(self, event):
		self.update_params()
		self.deduce_action_dependencies()
		self.render_graph()
		
	def update_params(self):
		param_names = ['graph_template', 'node_template', 'color_text_template',
									'highlight_keywords', 'show_only_highlighted', 'colorize_highlighted']
		for param in param_names:
			self.__dict__[param] = rospy.get_param('~{}'.format(param))
		
		# update color tables
		for keyword in self.highlight_keywords:
			if keyword not in self.color_table:
				hue = self.calculate_color_hue(len(self.color_table))
				self.color_table[keyword] = '{} {} {}'.format(hue, 1.0, 0.8)
			
	def calculate_color_hue(self, index):
		return (29 * float(index) % 360.0) / 360.0
	
	def colorize_text(self, text):
		if len(self.highlight_keywords) == 0:
			return text
		if text in self.color_table:
			if self.colorize_highlighted:
				return self.color_text_template.format(color=self.color_table[text], arg=text)
			else:
				return self.color_text_template.format(color='black', arg=text)
		else:
			if self.show_only_highlighted:
				return ''
			return self.color_text_template.format(color='grey', arg=text)
				
	
	def render_graph(self):
		if not self.plan:
			return
		
		# action status subgraphs
		enabled = ' '.join(str(i) for i in self.enabled_action_ids)
		achieved = ' '.join(str(i) for i in self.achieved_action_ids)
		failed = ' '.join(str(i) for i in self.failed_action_ids)
		
		# actions with highlighted key words
		node_list = []
		for action in self.plan:
			arg_list = [self.colorize_text(kv.value) for kv in action.parameters]
			args = ' '.join(arg_list)
			node = self.node_template.format(id=action.action_id, name=action.name, args=args)
			node_list.append(node)
		nodes = '\n'.join(node_list)
		
		# add graph connections
		edge_list = []
		for a1 in range(len(self.plan)):
			for a2 in range(len(self.plan)):
				if self.dependency_matrix[a1, a2]:
					edge_list.append('"{}" -> "{}"'.format(a2, a1))
		edges = '\n'.join(edge_list)

		graph = self.graph_template.format(
			enabled=enabled, achieved=achieved, failed=failed, nodes=nodes, edges=edges)
		rospy.loginfo(graph)
		src = Source(graph)
		src.render('plan.gv', view=True)

def parse_temporal_plan(plan_file):
	raw_plan = []
	with open(plan_file) as f:
		raw_plan = f.read()
		
	# typical PDDL raw_plan action
	# 193.02530000: (transport-product r3 p70 bs_out bs rs2_in rs2 silver_base_p70 blue_ring_p70) [58.66200000]
	action_re = "([0-9]*\\.?[0-9]+).*: *\\((.+)\\) *\\[([0-9]*\\.?[0-9]+)\\]"
	matches = re.findall(action_re, raw_plan)
	plan = CompletePlan()
	for match in matches:
		action = ActionDispatch()
		action.action_id = len(plan.plan)
		action.dispatch_time = float(match[0])
		action.duration = float(match[2])
		action_parts = match[1].split()
		action.name = action_parts[0]
		for parameter in action_parts[1:]:
			kv = KeyValue()
			kv.value = parameter
			action.parameters.append(kv)
		plan.plan.append(action)
	return plan

if __name__ == '__main__':
	rospy.init_node('vis_plan_graph', argv=sys.argv, anonymous=True)
	vis = Visualization()
	
	if len(sys.argv) > 1 and sys.argv[1][0:2] != '__':
		rospy.loginfo('visualizing offline plan: {}'.format(sys.argv[1]))
		vis.plan_callback(parse_temporal_plan(sys.argv[1]))
	else:
		rospy.loginfo('visualizing plans from rosplan topic')
		plan_subscriber = rospy.Subscriber(
			'/kcl_rosplan/plan', CompletePlan, vis.plan_callback)
		action_feedback_subscriber = rospy.Subscriber(
			'/kcl_rosplan/action_feedback', ActionFeedback, vis.action_feedback_callback)
		pass
	rospy.spin()


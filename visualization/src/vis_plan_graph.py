#!/usr/bin/env python
import sys

sys.path.append('/usr/lib/pyshared/python2.6')

import argparse

import rospy
import colorsys

from graphviz import Source

from rosplan_dispatch_msgs.msg import CompletePlan
from std_msgs.msg import String
from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan_dispatch_msgs.msg import ActionFeedback

class Visualization:
	def __init__(self):
		self.plan = []  # ActionDispatch[]
		self.graph = None  # string
		
		
		self.achieved_action_ids = set()
		self.enabled_action_ids = set()
		self.failed_action_ids = set()
		
		self.color_table = {}
		self.update_params()
		
		self.plan_subscriber = rospy.Subscriber(
			'/kcl_rosplan/plan', CompletePlan, self.plan_callback)
		self.graph_subscriber = rospy.Subscriber(
			'/kcl_rosplan/plan_graph', String, self.graph_callback)
		self.action_feed_back_subscriber = rospy.Subscriber(
			'/kcl_rosplan/action_feedback', ActionFeedback, self.action_feedback_callback)
		
		#rospy.sleep(0.3)
		#self.render_graph()
		self.timer = rospy.Timer(period=rospy.Duration(0.3), callback=self.schedule_update, oneshot=True)

	def plan_callback(self, msg):
		rospy.loginfo('plan')
		self.plan = msg.plan
		self.achieved_action_ids.clear()
		self.enabled_action_ids.clear()
		self.failed_action_ids.clear()
		self.timer = rospy.Timer(period=rospy.Duration(0.1), callback=self.schedule_update, oneshot=True)

	def graph_callback(self, msg):
		rospy.loginfo('graph')
		self.graph = msg.data
		
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
		
	def schedule_update(self, event):
		self.update_params()
		self.render_graph()
		
	def update_params(self):
		param_names = ['graph_template', 'node_template',
									'arg_template', 'highlight_keywords', 'show_only_highlighted']
		for param in param_names:
			self.__dict__[param] = rospy.get_param('~{}'.format(param))
		
		# update color tables
		for keyword in self.highlight_keywords:
			if keyword not in self.color_table:
				#color = colorsys.hsv_to_rgb(h=self.calculate_color_hue(len(self.color_table)), s=1.0, v=0.8)
				#self.color_table[keyword] = '{} {} {}'.format(color[0], color[1], color[2])
				self.color_table[keyword] = '{} {} {}'.format(self.calculate_color_hue(len(self.color_table)), 1.0, 0.8)
			
	def calculate_color_hue(self, index):
		return (29*float(index)%360.0)/360.0
			
	def render_graph(self):
		if not self.graph or not self.plan:
			return
		rospy.loginfo('start render graph')
		
		# action status subgraphs
		enabled = ' '.join(str(i) for i in self.enabled_action_ids)
		achieved = ' '.join(str(i) for i in self.achieved_action_ids)
		failed = ' '.join(str(i) for i in self.failed_action_ids)
		
		# actions with highlighted key words
		node_list = []
		for action in self.plan:
			arg_list = []
			if self.highlight_keywords and len(self.highlight_keywords) > 0:
				for kv in action.parameters:
					if kv.value in self.highlight_keywords:
						arg_list.append(self.arg_template.format(color=self.color_table[kv.value], arg=kv.value))
					elif not self.show_only_highlighted:
						arg_list.append(kv.value)
			else:
				arg_list = [kv.value for kv in action.parameters]
			args = ' '.join(arg_list)
			node = self.node_template.format(id=action.action_id, name=action.name, args=args)
			node_list.append(node)
		nodes = '\n'.join(node_list)
		
		# graph connections
		e_start = self.graph.find('"0"')
		e_end = self.graph.rfind('"')
		edges = self.graph[e_start:e_end+1]

		graph = self.graph_template.format(
			enabled=enabled, achieved=achieved, failed=failed, nodes=nodes, edges=edges)
		rospy.loginfo(graph)
		src = Source(graph)
		src.render('plan.gv', view=True)

def main(args):
	rospy.init_node('vis_plan_graph', anonymous=True)
	ic = Visualization()
	rospy.spin()


if __name__ == '__main__':

	main(sys.argv)

#!/usr/bin/env python
import rospy
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback

pending_actions = dict()
action_timers = dict()

def dispatch_cb(action):
	global pending_actions
	if action.action_id not in pending_actions:
		pending_actions[action.action_id] = action
		action_timers[action.action_id] = rospy.Time(), rospy.Time(), rospy.Time()
	else:
		rospy.loginfo("Recorded duplicate action id %d with action name %s", action.action_id, action.name)
	return

def feedback_cb(action):
	global pending_actions
	if action.action_id in pending_actions:
		now = rospy.Time.now()
		if action.status == "action_enabled":
			action_timers[action.action_id][0] = now
		else if action.status == "action_achieved":
			action_timers[action.action_id][1] = now
			del pending_action[action.action_id]
		else if action.status == "action_failed":
			action_timers[action.action_id][2] = now
			del pending_action[action.action_id]
		else:
			rospy.loginfo("Something's off. Received unknown action status %s for action id %d", action.action_status, action.action_id)
	else:
		rospy.loginfo("Something's off. Received feedback for an action whose dispatch wasn't recorded. Action id is %d", action.action_id)
	return

def listener():
	rospy.init_node('Timekeeper', anonymous=True)
	rospy.Subscriber("/kcl_rosplan/action_dispatch", ActionDispatch, dispatch_cb)
	rospy.Subscriber("/kcl_rosplan/action_feedback", ActionFeedback, feedback_cb)
	rospy.spin()



if __name__ == '__main__':
	listener()

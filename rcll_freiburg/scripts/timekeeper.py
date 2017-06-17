#!/usr/bin/env python
import rospy
import sys
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback

pending_actions = dict()
action_timers = dict()
output_file = ''

def record_time(action_id, status):
    of = open(output_file, 'a')
    dur = (action_timers[action_id][status] - action_timers[action_id][0]).to_sec()
    line = '{0}\t{1}\t{2}\t{3}\t{4}\t{5}'.format(pending_actions[action_id].name, action_id, action_timers[action_id][0], 'ACHVD' if status == 1 else 'FAILED', action_timers[action_id][status], dur)

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
        elif action.status == "action_achieved":
            action_timers[action.action_id][1] = now
            record_time(action.action_id, 1)
            del pending_action[action.action_id]
        elif action.status == "action_failed":
            action_timers[action.action_id][2] = now
            record_time(action.action_id, 2)
            del pending_action[action.action_id]
        else:
            rospy.loginfo("Something's off. Received unknown action status %s for action id %d", action.action_status, action.action_id)
    else:
        rospy.loginfo("Something's off. Received feedback for an action whose dispatch wasn't recorded. Action id is %d", action.action_id)
    return

def listener():
    rospy.init_node('Timekeeper', anonymous=True)
    if len(sys.argv) < 2:
        rospy.signal_shutdown('The output filename needs to be passed as an argument to this node')
    elif len(sys.argv) > 2:
        rospy.signal_shutdown('The node can only accept one argument input.')
    output_file = sys.argv[1]
    print 'Printing action timing information to file :', output_file
    of = open(output_file)
    of.write("NAME\tID\tDISPATCH\tSTATUS\tEND_TIME\tDUR")
    of.close()
    rospy.Subscriber("/kcl_rosplan/action_dispatch", ActionDispatch, dispatch_cb)
    rospy.Subscriber("/kcl_rosplan/action_feedback", ActionFeedback, feedback_cb)
    rospy.spin()



if __name__ == '__main__':
   listener()

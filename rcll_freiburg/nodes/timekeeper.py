#!/usr/bin/env python
import rospy
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback

pending_actions = dict()
action_timers = dict()
output_file = ''

def record_time(action_id, status):
    of = open(output_file, 'a')
    dur = (action_timers[action_id][status] - action_timers[action_id][0]).to_sec()
    line = '{0}\t{1}\t{2}\t{3}\t{4}\t{5}\n'.format(pending_actions[action_id].name, action_id, action_timers[action_id][0], 'ACHVD' if status == 1 else 'FAILED', action_timers[action_id][status], dur)
    of.write(line)
    of.close()
    return

def dispatch_cb(action):
    global pending_actions
    if action.action_id not in pending_actions:
        pending_actions[action.action_id] = action
	action_timers[action.action_id] = [rospy.Time(), rospy.Time(), rospy.Time()]
        msg = "[Timekeeper] recording action {0} with new id {1}".format(action.name, action.action_id)
        rospy.loginfo(msg)
    else:
	rospy.logwarn("[Timekeeper] Recorded duplicate action id {0} with action name {1}".format(action.action_id, action.name))
    return

def feedback_cb(action):
    global pending_actions
    if action.action_id in pending_actions:
        now = rospy.Time.now()
        if action.status == "action enabled":
            action_timers[action.action_id][0] = now
            msg = "[Timekeeper] Received action_enabled message for action id {0}".format(action.action_id)
            rospy.loginfo(msg)
        elif action.status == "action achieved":
            action_timers[action.action_id][1] = now
            msg = "[Timekeeper] Received action_achieved message for action id {0}".format(action.action_id)
            rospy.loginfo(msg)
            record_time(action.action_id, 1)
            del pending_actions[action.action_id]
        elif action.status == "action failed":
            action_timers[action.action_id][2] = now
            msg = "[Timekeeper] Received action_failed message for action id {0}".format(action.action_id)
            rospy.loginfo(msg)
            record_time(action.action_id, 2)
            del pending_actions[action.action_id]
        else:
            rospy.logwarn("[Timekeeper] Something's off. Received unknown action status {0} for action id {1}".format(action.status, action.action_id))
    else:
        rospy.logwarn("[Timekeeper] Something's off. Received feedback for an action whose dispatch wasn't recorded. Action id is {0}".format(action.action_id))
    return

def listener():
    rospy.init_node('Timekeeper')
    global output_file
    argv = rospy.myargv()
    if len(argv) < 2:
        rospy.signal_shutdown('[Timekeeper] The output filename needs to be passed as an argument to the node')
    elif len(argv) > 2:
        rospy.signal_shutdown('[Timekeeper] The node can only accept one argument input.')
    output_file = argv[1]
    if output_file == '':
        rospy.signal_shutdown('[Timekeeper] The node received an empty file name as output file, shutting down.')
    msg = '[Timekeeper] node successfully initialized, printing action timing information to file :', output_file
    rospy.loginfo(msg)
    of = open(output_file, 'w')
    of.write("NAME\tID\tDISPATCH\tSTATUS\tEND_TIME\tDUR\n")
    of.close()
    rospy.Subscriber("/kcl_rosplan/action_dispatch", ActionDispatch, dispatch_cb)
    rospy.Subscriber("/kcl_rosplan/action_feedback", ActionFeedback, feedback_cb)
    rospy.spin()
    rospy.loginfo("[Timekeeper] Shutting down node.")



if __name__ == '__main__':
    listener()

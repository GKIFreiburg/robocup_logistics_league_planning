#!/usr/bin/env python
import sys

sys.path.append('/usr/lib/pyshared/python2.6')

import rospy
from std_msgs.msg import String

from graphviz import Source

from rosplan_dispatch_msgs.msg import ActionDispatch


class Visualization:

    def __init__(self):
        self.graph_subscriber = rospy.Subscriber('/kcl_rosplan/plan_graph', String, self.graph_callback)
        self.action_subscriber = rospy.Subscriber('/kcl_rosplan/action_dispatch', ActionDispatch, self.action_callback)

        self.dotcode = None
        self.current_action = ""

    def graph_callback(self, data):
        if self.dotcode!= data.data:
            self.dotcode = data.data
            #self.set_color(self.current_action)
            self.update_graph()


    def action_callback(self, data):

        if self.current_action != data.name:
            self.current_action = data.name
            self.set_color(self.current_action)
            self.update_graph()


    def update_graph(self):
        src = Source(self.dotcode)
        src.render('test-output/holy-grenade.gv', view=True)

    def set_color(self, name):
        name = name+'_'
        print(name)
        index = self.dotcode.find(name)
        substring = self.dotcode[index:]
        index2 = substring.find("style=") + index

        self.dotcode = self.dotcode[:index2] + 'color=red ' + self.dotcode[index2:]
        print(self.dotcode)




def main(args):
    '''Initializes and cleanup ros node'''
    ic = Visualization()
    rospy.init_node('vis_plan_graph', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

#def callback(data):

#    src = Source(data.data)
#    src.render('test-output/holy-grenade.gv', view=True)




#def listener():
#    rospy.init_node('vis_plan_graph', anonymous=True)
#    rospy.Subscriber('/kcl_rosplan/plan_graph', String, callback)
#    rospy.spin()

#if __name__ == '__main__':
#    listener()

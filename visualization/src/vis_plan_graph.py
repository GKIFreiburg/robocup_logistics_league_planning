#!/usr/bin/env python
import sys

sys.path.append('/usr/lib/pyshared/python2.6')

import rospy
from std_msgs.msg import String

from graphviz import Source

from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan_dispatch_msgs.msg import ActionFeedback


class Visualization:

    def __init__(self):
        self.graph_subscriber = rospy.Subscriber('/kcl_rosplan/plan_graph', String, self.graph_callback)
        self.action_subscriber = rospy.Subscriber('/kcl_rosplan/action_dispatch', ActionDispatch, self.action_callback)
        self.action_feed_back_subscriber = rospy.Subscriber('/kcl_rosplan/action_feedback', ActionFeedback, self.action_feedback_callback)

        self.dotcode = ""
        self.current_action = ""
        self.current_action_id = 0

        self.enabled_actions = list()#[]
        self.failed_actions = list()#[]
        self.succeeded_actions = list()#[]

    def graph_callback(self, data):
        if self.dotcode!= data.data:
            self.dotcode = data.data
            self.set_color()
            self.update_graph()

            self.enabled_actions = list()#[]
            self.failed_actions = list()#[]
            self.succeeded_actions = list()#[]

    def action_feedback_callback(self, data):

        action_id = int(data.action_id)

        if 'enabled' in data.status:
            self.enabled_actions.append(action_id)

        elif 'achieved' in data.status:
            self.succeeded_actions.append(action_id)
            if data.action_id in self.enabled_actions:
                self.enabled_actions.remove(data.action_id)

        elif 'failed' in data.status:
            self.failed_actions.append(action_id)
            if data.action_id in self.enabled_actions:
                self.enabled_actions.remove(data.action_id)

        self.set_color()



    def action_callback(self, data):

        if self.current_action != data.name:
            self.current_action = data.name
            self.current_action_id = data.action_id
            self.set_color()
            self.update_graph()


    def update_graph(self):
        self.set_color()
        #print('----------------------NEW GRAPH----------------------')
        #print(self.dotcode)
        src = Source(self.dotcode)
        src.render('test-output/holy-grenade.gv', view=True)

    def set_color(self):

#        color_red = 'color=red '
#        while (color_red in self.dotcode):
#            self.dotcode = self.dotcode.replace(color_red, '')

#        color_blue = 'color=blue '
#        while (color_blue in self.dotcode):
#            self.dotcode = self.dotcode.replace(color_blue, '')

#        color_green = 'color=green '
#        while (color_green in self.dotcode):
#            self.dotcode = self.dotcode.replace(color_green, '')

        if len(self.enabled_actions) > 0:
            print('--------------------- enabled actions ---------------------')
        for action in self.enabled_actions:
            print action
            id = str(action) + '[ '
            index = self.dotcode.find(id)
            substring = self.dotcode[index:]
            index2 = substring.find("style=") + index
            self.dotcode = self.dotcode[:index2] + 'color=yellow ' + self.dotcode[index2:]

        if len(self.failed_actions) > 0:
            print('---------------------- failed actions ---------------------')
        for action in self.failed_actions:
            print action
            id = str(action) + '[ '
            index = self.dotcode.find(id)
            substring = self.dotcode[index:]
            index2 = substring.find("style=") + index
            self.dotcode = self.dotcode[:index2] + 'color=red ' + self.dotcode[index2:]

        if len(self.succeeded_actions) > 0:
            print('-------------------- succeeded actions --------------------')
        for action in self.succeeded_actions:
            print action
            id = str(action) + '[ '
            index = self.dotcode.find(id)
            substring = self.dotcode[index:]
            index2 = substring.find("style=") + index
            self.dotcode = self.dotcode[:index2] + 'color=green ' + self.dotcode[index2:]



        #print('-----------OLD GRAPH-----------')
        #print (self.dotcode)
        #x = 'color=red '
        #self.dotcode = self.dotcode.replace(x, '')
        #print('-----------OLD GRAPH DELETED-----------')
        #print (self.dotcode)
        #print(name)
        #name = name+'_'
        #id = str(id) + '[ '

        #index = self.dotcode.find(name)
        #index = self.dotcode.find(id)
        #substring = self.dotcode[index:]
        #index2 = substring.find("style=") + index


        #print('-----------SUB GRAPH-----------')
        #print (self.dotcode[:index2])
        #self.dotcode = self.dotcode[:index2] + 'color=red ' + self.dotcode[index2:]





def main(args):
    '''Initializes and cleanup ros node'''
    ic = Visualization()
    rospy.init_node('vis_plan_graph', anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

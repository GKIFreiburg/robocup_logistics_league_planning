#!/usr/bin/env python
import sys

sys.path.append('/usr/lib/pyshared/python2.6')

import argparse

import rospy
from std_msgs.msg import String

from graphviz import Source

from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan_dispatch_msgs.msg import ActionFeedback


class Visualization:

    def __init__(self):
        self.graph_subscriber = rospy.Subscriber('/kcl_rosplan/plan_graph', String, self.graph_callback)
        self.action_feed_back_subscriber = rospy.Subscriber('/kcl_rosplan/action_feedback', ActionFeedback, self.action_feedback_callback)

        self.dotcode = ""
        self.actions = list()

        self.enabled_actions = list()#[]
        self.failed_actions = list()#[]
        self.succeeded_actions = list()#[]

        self.succeeded_color = 'color=chartreuse2 penwidth=2.0 '
        self.enabled_color = 'color=darkgoldenrod1 penwidth=2.0 '
        self.failed_color = 'color=firebrick3 penwidth=2.0 '

        self.color_words =  rospy.get_param("/color_words")
        self.colors = list()

    def calculate_colors(self):
        self.colors = list()
        self.color_words =  rospy.get_param("/color_words")

        number = len(self.color_words)

        if number > 6:
            divider = 360.0 / 6
        else:
            divider = 360.0 / (number)

        for i in range(0,number):

            if i >= 6:
                i = i-6
                h =float(i * divider + 30)
            else:
                h = float(i * divider)

            h = h/360
            s = 1.0
            v = 0.6
            print h
            self.colors.append(str(h)+' '+str(s)+' '+str(v))


    def new_graph(self, new_dotcode):
        self.actions = list()

        counter = 0
        index = 0
        while index >= 0:
            find = str(counter) + '[ '
            index = self.dotcode.find(find)
            counter = counter +1

        for i in range (0, (counter-1)):
            find = str(i) + '[ '
            index = self.dotcode.find(find)
            if index > -1:
                substring = self.dotcode[index:]

                index2 = substring.find("label=") + index + 7
                substring = self.dotcode[index2:]

                index3 = substring.find("style=") + index2
                substring2 = self.dotcode[index2:(index3-2)]

                self.actions.append(substring2)

        new_actions = list()
        counter = 0
        index = 0
        while index >= 0:
            find = str(counter) + '[ '
            index = new_dotcode.find(find)
            counter = counter + 1

        if not((counter-1) == len(self.actions)):
            return True

        for i in range (0, (counter-1)):
            find = str(i) + '[ '
            index = new_dotcode.find(find)
            if index > -1:
                substring = new_dotcode[index:]

                index2 = substring.find("label=") + index + 7
                substring = new_dotcode[index2:]

                index3 = substring.find("style=") + index2
                substring2 = new_dotcode[index2:(index3-2)]
                new_actions.append(substring2)

        for i in range (0, len(self.actions)):
            if not(self.actions[i] == new_actions[i]):
                return True

        return False

    def set_font_color(self):

        self.calculate_colors()

        graph = self.dotcode

        counter = 0
        index = 0
        while index >= 0:
            find = str(counter) + '[ '
            index = graph.find(find)
            counter = counter +1

        for i in range (0, (counter-1)):
            find = str(i) + '[ '
            index = graph.find(find)

            if index > -1:
                substring = graph[index:]

                index2 = substring.find("label=") + index + 7
                substring = graph[index2:]

                index3 = substring.find("style=") + index2
                old_label = graph[index2:(index3-2)]
                tmp = old_label


                part1 = graph[:(index2-1)]
                part2 = graph[(index3-1):]

                number = old_label.count("\n")

                new_label = "<"
                last_index = 0
                for j in range(0, (number+1)):

                    index = old_label.find("\n")

                    if j == number:
                        find_label = old_label

                    else:
                        find_label = old_label[:index]


                    if (find_label in self.color_words):

                        color_index = self.color_words.index(find_label)

                        if (color_index >= 0):
                            color = str(self.colors[color_index])
                            color = '"'+str(color)+'"'

                            if j == number:
                                new_label = new_label + """<FONT color="""+color+""">""" + old_label + """</FONT> <br/> """
                            else:
                                new_label = new_label + """<FONT color="""+color+""">""" + old_label[:index] + """</FONT> <br/> """

                    else:
                        if j == number:
                            new_label = new_label + """<FONT color="grey42">""" + old_label + """</FONT> <br/> """
                        else:
                            new_label = new_label + """<FONT color="grey42">""" + old_label[:index] + """</FONT> <br/> """

                    old_label = old_label[(index+1):]

                new_label = new_label + ">"
                index = tmp.find("\n")

                graph = part1 + new_label + part2

        return graph




    def graph_callback(self, data):

        set = self.new_graph(data.data)

        if set:

            self.dotcode = data.data

            self.enabled_actions = list()
            self.failed_actions = list()
            self.succeeded_actions = list()

            self.set_color()


    def action_feedback_callback(self, data):

        action_id = int(data.action_id)

        if 'enabled' in data.status:
            self.enabled_actions.append(action_id)
            self.enabled_actions = list(set(self.enabled_actions))

            if data.action_id in self.failed_actions:
                self.failed_actions.remove(data.action_id)

            if data.action_id in self.succeeded_actions:
                self.succeeded_actions.remove(data.action_id)

        elif 'achieved' in data.status:
            self.succeeded_actions.append(action_id)
            self.succeeded_actions = list(set(self.succeeded_actions))

            if data.action_id in self.enabled_actions:
                self.enabled_actions.remove(data.action_id)

            if data.action_id in self.failed_actions:
                self.failed_actions.remove(data.action_id)

        elif 'failed' in data.status:
            self.failed_actions.append(action_id)
            self.failed_actions = list(set(self.failed_actions))

            if data.action_id in self.enabled_actions:
                self.enabled_actions.remove(data.action_id)

            if data.action_id in self.succeeded_actions:
                self.succeeded_actions.remove(data.action_id)

        self.set_color()



    def set_color(self):

        graph = self.dotcode

        graph = self.set_font_color()

        for action in self.enabled_actions:
            id = str(action) + '[ '
            index = graph.find(id)
            if index > -1:
                substring = graph[index:]
                index2 = substring.find("style=") + index
                graph = graph[:index2] + self.enabled_color + graph[index2:]

        for action in self.failed_actions:
            id = str(action) + '[ '
            index = graph.find(id)
            if index > -1:
                substring = graph[index:]
                index2 = substring.find("style=") + index
                graph = graph[:index2] + self.failed_color + graph[index2:]

        for action in self.succeeded_actions:
            id = str(action) + '[ '
            index = graph.find(id)
            if index > -1:
                substring = graph[index:]
                index2 = substring.find("style=") + index
                graph = graph[:index2] + self.succeeded_color + graph[index2:]

        src = Source(graph)
        src.render('test-output/holy-grenade.gv', view=True)


def main(args):
    '''Initializes and cleanup ros node'''
    ic = Visualization()
    rospy.init_node('vis_plan_graph', anonymous=True)
    rospy.spin()


if __name__ == '__main__':

    main(sys.argv)

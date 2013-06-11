#!/usr/bin/env python

import roslib 
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
import rospy
import random
from actionlib import SimpleActionServer
from the_curious_robot.msg import ArticulateOOIAction

class ArticulateOOIActionServer:
    def __init__(self, name):
        self.server = SimpleActionServer(name, ArticulateOOIAction,
                execute_cb=self.execute)
        self.server.start()

    def execute(self, msg):
        rospy.sleep(.33)
        rospy.logdebug('Articulate object of interest')
        r = random.random()
        if r > .5:
            self.server.set_succeeded()
        else:
            self.server.set_aborted()

def main():
    rospy.init_node('tcr_sas_articulate_ooi')
    server = ArticulateOOIActionServer('articulate_ooi')

if __name__ == '__main__':
    main()
    rospy.spin()

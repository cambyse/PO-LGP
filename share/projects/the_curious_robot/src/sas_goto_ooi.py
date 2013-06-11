#!/usr/bin/env python

import roslib
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
import rospy
from actionlib import SimpleActionServer
from the_curious_robot.msg import GotoOOIAction

class GotoOOIActionServer:
    def __init__(self, name):
        self.server = SimpleActionServer(name, GotoOOIAction,
                execute_cb=self.execute)
        self.server.start()

    def execute(self, msg):
        rospy.sleep(.33);
        rospy.logdebug('Goto object of interest')
        self.server.set_succeeded()

def main():
    rospy.init_node('tcr_sas_goto_ooi')
    server = GotoOOIActionServer('goto_ooi')

if __name__ == '__main__':
    main()
    rospy.spin()

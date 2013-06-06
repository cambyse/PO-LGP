#!/usr/bin/env python

import roslib
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
import rospy
from actionlib import SimpleActionServer
from the_curious_robot.msg import ObserveOOITrajAction

class ObserveOOITrajActionServer:
    def __init__(self, name):
        self.server = SimpleActionServer(name, ObserveOOITrajAction,
                execute_cb=self.execute)
        self.server.start()

    def execute(self, msg):
        rospy.sleep(.33)
        rospy.logdebug('Observe object of interest trajectory')
        self.server.set_succeeded()

def main():
    rospy.init_node('tcr_sas_observe_ooi_traj', log_level=rospy.DEBUG)
    server = ObserveOOITrajActionServer('observe_ooi_traj')

if __name__ == '__main__':
    main()
    rospy.spin()

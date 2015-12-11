#!/usr/bin/env python

import roslib
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
import rospy
import the_curious_robot.msg as msgs
import the_curious_robot.srv as srvs
import geometry_msgs.msg
import random


class GotoRandomMsg:

    def __init__(self):
        rospy.init_node('tcr_goto_random', log_level=rospy.DEBUG)
        self.control_done = False

        # subscriber
        self.control_done_sub = rospy.Subscriber(
            'control_done',
            msgs.control_done, self.control_done_cb)

        # publisher
        self.control_pub = rospy.Publisher('control', msgs.control)

    def control_done_cb(self, msg):
        rospy.logdebug("send new goal")
        msg = msgs.control()
        pos = geometry_msgs.msg.Vector3(random.uniform(-6, 6),
                                        random.uniform(-6, 6),
                                        1)
        msg.pose.translation = pos
        self.control_pub.publish(msg)
        rospy.logdebug("sent new goal")

    def run(self):
        rospy.sleep(3)
        self.control_done_cb(None)
        rospy.spin()


class GotoRandomSrv:
    def __init__(self):
        rospy.init_node('tcr_goto_random', log_level=rospy.DEBUG)

    def run(self):
        while not rospy.is_shutdown():
            self.step()

    def step(self):
        msg = srvs.ControlRequest()
        pos = geometry_msgs.msg.Vector3(random.uniform(-6, 6),
                                        random.uniform(-6, 6),
                                        1)
        msg.pose.translation = pos

        try:
            srv = rospy.ServiceProxy("control", srvs.Control())
            srv(msg)
        except rospy.ServiceException:
            pass


def main():
    s = GotoRandomMsg()
    s.run()


if __name__ == '__main__':
    main()

#!/usr/bin/env python2

import rospy
import the_curious_robot.msg as msgs

class Require:
    def __init__(self, modules):
        self.modules = dict(map(lambda x: (x, False), modules))
        self.wait = True

        self.alive_sub = rospy.Subscriber("alive", msgs.Alive, self.alive_cb)
        self.is_alive_pub = rospy.Publisher("is_alive", msgs.Alive)

        while self.wait:
            for module in self.modules:
                if not self.modules[module]:
                    msg = msgs.Alive()
                    msg.module = module
                    self.is_alive_pub.publish(msg)
                    rospy.logdebug("wait for " + module);
            rospy.sleep(.1)

    def alive_cb(self, msg):
        self.modules[msg.module] = True
        self.wait = not all(self.modules.values())

class Provide:
    def __init__(self, module):
        self.module = module
        self.is_alive_sub = rospy.Subscriber("is_alive", msgs.Alive,
                self.is_alive_cb)
        self.alive_pub = rospy.Publisher("alive", msgs.Alive)

    def is_alive_cb(self, msg):
        if msg.module == self.module:
            msg = msgs.Alive()
            msg.module = self.module
            self.alive_pub.publish(msg)

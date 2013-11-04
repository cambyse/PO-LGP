#!/usr/bin/env python
# encoding: utf-8

import roslib
roslib.load_manifest("rosors")
import rospy
import rosors.srv
import sys


def request_bodies():
    srv_name = '/prefix/bodies'
    print srv_name
    rospy.wait_for_service(srv_name)
    try:
        request_bodies = rospy.ServiceProxy(srv_name,
                                            rosors.srv.Bodies)
        if len(sys.argv) == 2:
            resp = request_bodies(name=sys.argv[1])
        else:
            resp = request_bodies()
        return resp

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == "__main__":
    resp = request_bodies()
    print len(resp.bodies)
    for b in resp.bodies:
        print "=" * 60
        print b.name,  type(b)
        print b.pos

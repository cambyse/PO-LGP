#! /usr/bin/python

import roslib; roslib.load_manifest('floornav')
import rospy

import actionlib
import geometry_msgs.msg as geom
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction

import sys

if __name__ == '__main__':


        rospy.init_node("floornav_client")
        target="move_base"
        client = actionlib.SimpleActionClient(target, MoveBaseAction)

        if not client.wait_for_server(rospy.Duration.from_sec(1.0)):
            print "No %s action server connection, stopping" % target
            sys.exit(-1)

        # relative goal in the base_link frame
        goal = MoveBaseGoal(
                target_pose=geom.PoseStamped(
                        header=Header(frame_id='base_link', stamp=rospy.get_rostime()),
                        pose=geom.Pose(
                            position=geom.Point(x=-0.5),
                            orientation=geom.Quaternion(w=1.0)
                        )
                )
        )
        client.send_goal(goal)
        print client.wait_for_result(rospy.Duration.from_sec(5.0))
        print client.get_state()

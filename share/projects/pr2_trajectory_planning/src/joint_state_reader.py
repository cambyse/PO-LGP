import rospy
from sensor_msgs.msg import JointState

"""
helper script that prints joint states
"""

joint_names = [
    "l_shoulder_pan_joint",
    "l_shoulder_lift_joint",
    "l_upper_arm_roll_joint",
    "l_elbow_flex_joint",
    "l_forearm_roll_joint",
    "l_wrist_flex_joint",
    "l_wrist_roll_joint",
]


def callback(data):
    print data.name
    for i, name in enumerate(data.name):
        if name in joint_names:
            print "%s: %f" % (name, data.position[i])
    print "=" * 80


def listener():
    rospy.init_node('joint_states_reader')
    rospy.Subscriber("/joint_states", JointState, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()

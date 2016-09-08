import rospy
import baxter_interface

from baxter_interface import CHECK_VERSION

from baxter_pykdl import baxter_kinematics

LEFT = 1
RIGHT = 2
PARTS = {
	LEFT: ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2'], 
	RIGHT: ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
}

def map_angles(part, pos):
	res = {}
	for i, val in enumerate(pos):
		res[PARTS[part][i]] = val
	print res
	return res

def main():
	rospy.init_node('baxter_kinematics')
	kin = baxter_kinematics('right')

	pos = [-0.25, 0.0, 0.0]
	angles = kin.inverse_kinematics(pos)
	print angles

	#right = baxter_interface.Limb('right')
	#right.move_to_joint_positions(map_angles(RIGHT, angles))

if __name__ == "__main__":
    main()

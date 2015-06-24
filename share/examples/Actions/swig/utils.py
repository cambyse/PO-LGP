import numpy as np

###############################################################################
# Sides
LEFT = 256
RIGHT = 255

DEFAULT_SIDE = LEFT

print("=" * 70)
print("DEFAULT_SIDE is set to {}".format("LEFT"))
print("  You can always change it: `DEFAULT_SIDE = RIGHT`")


###############################################################################
def assert_valid_shapes(name, shapes):
    if name not in shapes:
        raise ValueError("The given shape {} is not an existing shape"
                         .format(name))


###############################################################################
# Helper: symbols...
def strip_specs(fact):
    return fact[: fact.find(")")] + ")"


def conv_symbol(symbol):
    return '(conv ' + symbol[1:]


def pos_str2arr(pos_str):
    return np.array([float(i) for i in pos_str[2:-2].split(" ")])


def side2endeff(side=None):
    if side is None:
        side = DEFAULT_SIDE
    eff = {LEFT: "endeffL", RIGHT: "endeffR"}
    return eff[side]


def side2gripper_joint(side=None):
    if side is None:
        side = DEFAULT_SIDE
    gripper_joint = {LEFT: "l_gripper_joint", RIGHT: "r_gripper_joint"}
    return gripper_joint[side]


def side2wrist_joint(side=None):
    if side is None:
        side = DEFAULT_SIDE
    wrist_joint = {LEFT: "l_wrist_roll_joint", RIGHT: "r_wrist_roll_joint"}
    return wrist_joint[side]

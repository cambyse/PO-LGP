class Object(object):
    # OBJECT TYPE
    STATIC = 1
    MOVABLE = 2
    # JOINT TYPE
    NIL = 3
    ROT = 4
    PRIS = 5

    def __init__(self, object_type=STATIC, joint_type=NIL):
        self.object_type = object_type
        self.joint_type = joint_type

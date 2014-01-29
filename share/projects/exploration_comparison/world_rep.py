class Object(object):
    # OBJECT TYPE
    # - "static"
    # - "movable"
    # JOINT TYPE
    # - "nil"
    # - "rot"
    # - "pris"

    def __init__(self, name, object_type="static", joint_type="nil"):
        self.name = name
        self.object_type = object_type
        self.joint_type = joint_type

    def interact(self):
        """Interact and return the observations."""
        return self.object_type, self.joint_type

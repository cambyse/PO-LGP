"""
This module contains all actions.

"""
from swiginterface import interface


def open_gripper(side):
    pass


def close_gripper(side):
    pass


def touch(obj):
    id_ = interface.addTask("",
                            "",
                            "",
                            "")
    interface.activate_and_wait(id_)


def move_to(obj):
    pass


def hover_above(obj):
    pass


if __name__ == '__main__':
    open_gripper("left")
    touch("table")

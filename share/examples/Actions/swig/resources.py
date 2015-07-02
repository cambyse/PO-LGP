"""
A simple and convenient python interface for the ActivityMachine.

::

    ipython -i minimal.py

"""
from __future__ import print_function
from collections import namedtuple
import time
import signal
import numpy as np

import swig


# def signal_handler(signal, frame):
#     print("\n"*10)
#     print("\nYou pressed Ctrl+C!")
#     print("Stopping all facts")
#     new_facts = []
#     for fact in facts():
#         print("\n"*10)
#         print(fact)
#         if "conv" not in fact:
#             new_facts.append("(conv " + fact[1: fact.find(")") + 1])
#     for fact in new_facts:
#         interface.setFact(fact)


interface = swig.ActionSwigInterface()

# new convenient symbols
for s in ["rot", "qItself", "pos", "front", "gazeAt", "gripper", "align"]:
    interface.createNewSymbol(s)

# don't abort the swig interface on Ctr-C
# the signal must be geristered after the swig interface was initialized
# signal.signal(signal.SIGINT, signal_handler)


###############################################################################
# Helper: access ors structures
def shapes(name=None):
    return interface.getShapeByName(name) if name else interface.getShapeList()


def joints(name=None):
    return interface.getJointByName(name) if name else interface.getJointList()


def bodies(name=None):
    return interface.getBodyByName(name) if name else interface.getBodyList()


def facts():
    return interface.getFacts()



###############################################################################
# Convenient access and autocompletion to shapes, joints, and bodies
# Just type `s.<tab>` to get a list of all shapes
_tmp = list(shapes())
Shapes = namedtuple("Shapes", " ".join(_tmp))
s = Shapes(*_tmp)

_tmp = list(bodies())
Bodies = namedtuple("Bodies", " ".join(_tmp))
b = Bodies(*_tmp)

_tmp = list(joints())
Joints = namedtuple("Joints", " ".join(_tmp))
j = Joints(*_tmp)

del _tmp

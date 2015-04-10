import sys
import os
sys.path.append(os.path.join(os.path.expanduser("~"),'git/mlr/share/src/Swig/'))
import swig
S = swig.ActionSwigInterface(0)
#S.defineNewTaskSpaceControlAction("positionHand",({"type":"pos"}), ({"ref1":"endeffL"}), ({"target":"[.7, .3, .7]"}), ({"PD" :"[.5, .9, .1, 10.]"}));
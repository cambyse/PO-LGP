import sys
import os
sys.path.append(os.path.join(os.path.expanduser("~"),'git/mlr/share/src/Actions/'))
import swig
S = swig.ActionSwigInterface(0)

parameters=({"type":"pos", "ref1":"endeffL" , "target":"[.7, .3, .7]","PD" :"[.5, .9, .1, 10.]"})
S.defineNewTaskSpaceControlAction("positionHand", parameters)
S.startActivity(S.lit(["positionHand"]), parameters)
S.waitForCondition(S.lit(["positionHand", "conv"]));
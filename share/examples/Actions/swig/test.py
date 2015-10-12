import sys
import os
import time
sys.path.append(os.path.join(os.path.expanduser("~"),'git/mlr/share/src/Actions/'))
import swig
S = swig.ActionSwigInterface(0)
body = S.getBodyByName("handle")
S.getBodyByName("handle")
time.sleep(1.1)
parameters=({"type":"pos", "ref1":"endeffL" , "target":body["pos"],"PD" :"[.5, .9, .1, 10.]"})
S.defineNewTaskSpaceControlAction("positionHandL", parameters)
S.getBodyByName("handle")
S.startActivity(S.lit(["positionHandL"]), parameters)
S.getBodyByName("handle")

#S.waitForCondition(S.lit(["positionHandL", "conv"]));

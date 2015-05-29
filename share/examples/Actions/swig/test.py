import sys
import os
import time
sys.path.append(os.path.join(os.path.expanduser("~"),'git/mlr/share/src/Actions/'))
import swig
S = swig.ActionSwigInterface(0)
body = S.getBodyByName("handle")
#time.sleep(0.1)
parameters=({"type":"pos", "ref1":"endeffL" , "target":body["pos"],"PD" :"[.5, .9, .1, 10.]"})
S.defineNewTaskSpaceControlAction("positionHandL", parameters)
S.startActivity(S.lit(["positionHandL"]), parameters)

S.waitForCondition(S.lit(["positionHandL", "conv"]));

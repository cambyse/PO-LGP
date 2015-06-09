import sys
import os
import time
sys.path.append(os.path.join(os.path.expanduser("~"),'git/mlr-staff/share/src/Actions/'))
import swig
S = swig.ActionSwigInterface(0)
#body = S.getBodyByName("handle")

parameters=({"type":"pos", "ref1":"endeffR" , "target":"[.2, .4, .6]","PD" :"[.5, .9, .1, 10.]"})
S.defineNewTaskSpaceControlAction("positionHandR", ["FollowReferenceActivity"], parameters)
S.startActivity(["positionHandR"])
#S.waitForCondition(["conv", "positionHandR"]);
parameters=({"type":"homing", "ref1":"endeffL" , "target":"[.7, .3, .9]","PD" :"[.5, .9, .1, 10.]"})
S.defineNewTaskSpaceControlAction("positionHand2", ["FollowReferenceActivity"], parameters)
S.startActivity(["positionHand2"])
time.sleep(1.1)
S.Cancel()

#S.waitForCondition(["conv", "positionHand2"]);

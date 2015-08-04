import sys
import os
import time
sys.path.append(os.path.abspath('../../../src/Actions'))
import swig
S = swig.ActionSwigInterface(0)
#body = S.getBodyByName("handle")


#parameters=({"type":"qItself", "ref1":"l_gripper_joint" , "target":"[.001]","PD" :"[.5, .9, .1, 1.]"})
#S.defineNewTaskSpaceControlAction("open", ["FollowReferenceActivity"], parameters)
#S.startActivity(["open"])
#S.waitForCondition(["conv", "open"]);

parameters=({"type":"pos", "ref1":"endeffR" , "ref2":"marker17", "target":"[.0, .0, .0]","PD" :"[.5, .9, .1, 10.]"})
S.defineNewTaskSpaceControlAction("boxHandR", ["FollowReferenceActivity"], parameters)
S.startActivity(["markerHandR"])
S.waitForCondition(["conv", "markerHandR"]);

#parameters=({"type":"qItself", "ref1":"l_gripper_joint" , "target":"[.01]","PD" :"[.5, .9, .1, 1.]"})
#S.defineNewTaskSpaceControlAction("close", ["FollowReferenceActivity"], parameters)
#S.startActivity(["close"])
#S.waitForCondition(["conv", "close"]);
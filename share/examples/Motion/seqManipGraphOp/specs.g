Include = '../../../data/keywords.g'
Include = 'model.kvg'

KOMO{
  T = 100
  duration = 5
}

(MinSumOfSqr qItself){ order=2 time=[0 1] Hmetric=1e-1 }
#(EqualZero posDiff endeff target){ vec1=[.15 0 0] time=[1 1] scale=100 }
#(MinSumOfSqr pos endeff){ order=1 time=[1 1] scale=100 }
#(EqualZero vecAlign endeff target){ vec1=[1 0 0] vec2=[1 0 0] time=[1 1] target=[1] }
#(LowerEqualZero collisionIneq){ margin=0.05 scale=.1 }

(MakeJoint rigidZero graspRef obj1){ timeOfApplication=.49 }
(MakeJoint delete table obj1){ timeOfApplication=.49 }

#  ors::KinematicSwitch *op1 = new ors::KinematicSwitch();
#  op1->symbol = ors::KinematicSwitch::addJointZero;
#  op1->jointType = ors::JT_fixed;
#  op1->timeOfApplication = pickTime;
#  op1->fromId = G.getShapeByName("graspRef")->index;
#  op1->toId = G.getShapeByName("obj1")->index;
#  MP.switches.append(op1);

#  ors::KinematicSwitch *op2 = new ors::KinematicSwitch();
#  op2->symbol = ors::KinematicSwitch::deleteJoint;
#  op2->timeOfApplication = pickTime;
#  op2->fromId = G.getShapeByName("table")->index;
#  op2->toId = G.getShapeByName("obj1")->index;
#  MP.switches.append(op2);

#  //-- setup new motion problem
#  ors::Shape *grasp = G.getShapeByName("graspRef");
#  ors::Shape *obj = G.getShapeByName("obj1");
#  ors::Shape *tar = G.getShapeByName("target");

#reach:
(MinSumOfSqr posDiff graspRef obj1){ time=[0.49 0.5] scale=1e3 }
(MinSumOfSqr quatDiff graspRef obj1){ time=[0.49 0.5] scale=1e3 }
(MinSumOfSqr qItself){ order=1 time=[0.49 0.5] scale=1e1 }

#move up:
(MinSumOfSqr pos obj1){ order=1 scale=1e1 time=[0.53 0.55] target=[0 0 .5] }

#place:
(MinSumOfSqr posDiff obj1 target){ time=[1 1] scale=1e3 }
(MinSumOfSqr qItself){ order=1 time=[1 1] scale=1e1 }
(MinSumOfSqr quatDiff obj1 target){ time=[1 1] target=[1 0 0 0] scale=1e3 }

#rigid grasp:
(MinSumOfSqr qItself graspJoint){ order=1 time=[0.51 .99] scale=1e3 }

#(MinSumOfSqr collisionExceptPairs obj1 graspRef obj1 endeff obj1 table){ margin=.05 }
#(MinSumOfSqr collisionPairs obj1 table){ time=[.6 1] margin=.05 }

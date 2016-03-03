Include = '../../../data/keywords.g'
Include = 'model.kvg'

KOMO{
  T = 40
  duration = 7
  phases = 3
  makeConvexHulls, activateAllContacts, makeSCBoxes
}

#transition costs
(MinSumOfSqr qItself){ order=2 time=[0 3] }

#watch
#(MinSumOfSqr gazeAt endeffKinect obj1){ time=[.6 2.4] scale=1e-1 }
#(MinSumOfSqr gazeAt endeffLaser obj1){ time=[.6 2.4] scale=1e0 }

#kinematic switches
(MakeJoint delete table obj1){ time=1 }
(MakeJoint transXYPhiAtFrom table obj1){ time=1 }
(MakeJoint delete table obj1){ time=2 }
(MakeJoint rigidAtFrom table obj1){ time=2 }


#reach:
#(MinSumOfSqr GJK endeff obj1){ time=[1 1] target=[-.01 0 0] scale=100 }
(MinSumOfSqr posDiff endeffWorkspace obj1){ time=[1 1] scale=1e1 }
(MinSumOfSqr posDiff endeff obj1){ time=[1 1] target=[-.1 0 0] scale=100 }
(MinSumOfSqr posDiff endeff obj1){ order=1 scale=1e0 time=[.8 1] target=[.2 0 0] }
#(MinSumOfSqr posDiff graspRef obj1){ time=[1 1] scale=1e3 }
#(MinSumOfSqr quatDiff graspRef obj1){ time=[1 1] scale=1e3 }
(MinSumOfSqr qItself){ order=1 time=[0.97 1.03] scale=1e1 }

#push:
(MinSumOfSqr posDiff endeff obj1){ time=[1 2] target=[-.1 0 0] scale=100 }
(MinSumOfSqr pos obj1){ order=1 scale=1e0 time=[1 2] target=[.07 0 0] }
(MinSumOfSqr qItself){ order=1 time=[1.97 2.03] scale=1e1 }


#move up:
#(MinSumOfSqr pos obj1){ order=1 scale=1e0 time=[1 1.2] target=[0 0 .3] }

##move down:
#(MinSumOfSqr pos obj1){ order=1 scale=1e0 time=[1.8 2] target=[0 0 -.3] }

##place:
#(MinSumOfSqr qItself){ order=1 time=[1.97 2.03] scale=1e2 }
#(MinSumOfSqr posDiff obj1 target){ time=[2 2] scale=1e3 }
#(MinSumOfSqr posDiff endeffWorkspace target){ time=[2 2] scale=1e1 }
#(EqualZero quatDiff obj1 target){ time=[2 2] scale=1e3 }

##home:
(MinSumOfSqr posDiff endeff){ time=[3 3] scale=1e3 target=[1 -1 1.5] }
(MinSumOfSqr qItself){ order=1 time=[2.95 3] scale=1e1 }

##rigid grasp:
#(EqualZero qItself graspJoint){ order=1 time=[1 2] scale=1e3 }

##release

(LowerEqualZero collisionExcept obj1 r_wrist_roll_link obj1 graspRef obj1 endeff obj1 table obj1 table2 table table2){ margin=.05 time=[0 3] }
#(LowerEqualZero collisionPairs obj1 table){ time=[1.2 3] margin=.1 }
##(LowerEqualZero collisionPairs obj1 table2){ time=[0 .65] margin=.1 }
(LowerEqualZero limitIneq){ scale=1e0 time=[0 3] }



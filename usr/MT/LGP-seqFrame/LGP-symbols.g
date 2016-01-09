Include = '../../../share/data/keywords.g'
Include = 'LGP-world.g'

FOL_World{
  hasWait=false
}

Terminate
QUIT

#symbols
Object
Gsupport
articulated
loaded
free
#hand
touch
eff
holding
supports

Board
Cylin

#objects
table1
table2
obj1
#a
#b

START_STATE{
  (Board table1)
  (Board table2)
  (Object obj1)
  (Board obj1)
# (Object a) (Cylin a)
# (Object a) (Cylin b)
#  (articulated eff) (free eff)
  (articulated  graspRef) (free graspRef)
}

REWARD{}

DecisionRule Pick {
     Hand, Obj
     { (articulated Hand) (Object Obj) (free Hand) }
     { (articulated Obj) (holding Hand Obj) (free Hand)! (free Obj) }
}

DecisionRule Release {
     Hand, Obj, Onto
     { (articulated Obj) (Object Obj) (holding Hand Obj) (Board Onto) }
     { (articulated Obj)! (free Hand) (holding Hand Obj)! (supports Onto Obj)}
}

# =============================================================================

EffectiveKinematicsRule {
     Hand, Obj
     { (Pick Hand Obj) }
     { (EqualZero GJK Hand Obj){ scale=100 }
       (MakeJoint delete Obj)
       (MakeJoint rigid Hand Obj)
     }
}

EffectiveKinematicsRule {
     Hand, Obj, Onto
     { (Release Hand Obj Onto) }
     { (EqualZero GJK Obj Onto){ target=[0 0 .05] scale=100 }
       (MinSumOfSqr posDiff Obj Onto){ target=[0 0 .5] scale=10 }
       (MakeJoint delete Hand Obj)
       (MakeJoint transXYPhi Onto Obj)
       (MinSumOfSqr vec Obj){ vec1=[0 0 1] target=[0 0 1] scale=100}
     }
}

# =============================================================================

SeqProblemRule {
  Hand, Obj
  { (Pick Hand Obj) }
  { (MinSumOfSqr qItself){ order=1 time=[0.1 1] scale=1e0 }
    (MinSumOfSqr qZeroVels){ order=1 time=[0 1] scale=1e3 }
    (MinSumOfSqr posDiff Hand Obj){ time=[1 1] scale=1e3 }
    (MinSumOfSqr quatDiff Hand Obj){ time=[1 1] scale=1e3 }
    (MakeJoint delete Obj){ time=1 }
#    (MakeJoint rigidZero Hand Obj){ time=1 }
  }
}

SeqProblemRule {
  Hand, Obj, Onto
  { (Release Hand Obj Onto) }
  {
#    (MakeJoint delete Hand Obj){ time=0 }
    (MakeJoint transXYPhiZero Onto Obj){ time=0 from=<T t(0 0 .3)> }
    (MinSumOfSqr qItself){ order=1 time=[0.1 1] scale=1e0 }
    (MinSumOfSqr qZeroVels){ order=1  time=[0 1] scale=1e3 }
    (MinSumOfSqr posDiff Hand Obj){ time=[1 1] scale=1e3 }
    (MinSumOfSqr quatDiff Hand Obj){ time=[1 1] scale=1e3 }
#    (EqualZero GJK Obj Onto){ target=[0 0 .05] scale=100 }
    (MinSumOfSqr posDiff Obj Onto){ time=[1 1] target=[0 0 .2] scale=1e-1 } #1/2 metre above the thing
#    (MinSumOfSqr vec Obj){ time=[1 1] vec1=[0 0 1] target=[0 0 1] scale=100} #upright
  }
}

#SeqProblemRule {
#  Hand, Obj, Onto
#  { (Release Hand Obj Onto) }
#  { (MinSumOfSqr qItself){ order=1 time=[0.1 1] scale=1e0 }
#    (MinSumOfSqr qZeroVels){ order=1 scale=1e2 }
##    (EqualZero GJK Obj Onto){ target=[0 0 .05] scale=100 }
#    (MinSumOfSqr posDiff Obj Onto){ time=[1 1] target=[0 0 .2] scale=1 } #1/2 metre above the thing
#    (MinSumOfSqr vec Obj){ time=[1 1] vec1=[0 0 1] target=[0 0 1] scale=100} #upright
#    (MakeJoint delete Hand Obj){ time=1 }
#    (MakeJoint transXYPhi Onto Obj){ time=1 }
#  }
#}

# =============================================================================

PathProblemRule {
  Hand, Obj
  { (Pick Hand Obj) }
  { _MinSumOfSqr_qItself(MinSumOfSqr qItself){ time=[0 1] order=2 Hmetric=1e-1 } #transitions
    #(EqualZero GJK Hand Obj){ time=[1 1] scale=100 } #touch is not necessary
    _MinSumOfSqr_qItself_vel(MinSumOfSqr qItself){ order=1 time=[0.98 1] scale=1e1 } #slow down
    (MinSumOfSqr posDiff Hand Obj){ time=[.98 1] scale=1e3 }
    (MinSumOfSqr quatDiff Hand Obj){ time=[.98 1] scale=1e3 }

    (MakeJoint delete Obj){ time=1 }
    (MakeJoint rigidZero Hand Obj){ time=1 }
  }
}

PathProblemRule {
  Hand, Obj, Onto
  { (Release Hand Obj Onto) }
  { (MinSumOfSqr qItself){ time=[0 1] order=2 Hmetric=1e-1 } #transitions
  #    (EqualZero GJK Obj Onto){ time=[1 1] target=[0 0 .05] scale=100 } #touch
    (MinSumOfSqr pos Obj){ order=1 scale=1e-1 time=[0 0.15] target=[0 0 .1] } # move up
#    (MinSumOfSqr qItself){ order=1 time=[0.9 1] scale=1e1 } #slow down
    (MinSumOfSqr posDiff Obj Onto){ time=[1 1] target=[0 0 .2] scale=1000 } #1/2 metre above the thing
    (MinSumOfSqr vec Obj){ time=[1 1] vec1=[0 0 1] target=[0 0 1] scale=100} #upright
    (MakeJoint delete Hand Obj){ time=1 }
    (MakeJoint rigid Onto Obj){ time=1 }
  }
}

# =============================================================================
#EffectiveKinematicsRule {
#     X, Y
#     { (touch X Y) }
#     { (EqualZero GJK X Y) }
#}

#noDecisionRule TouchObject {
#     X
#     { (free hand) (Object X) }
#     { (free hand)! (touch eff X) (articulated X) }
#}

noDecisionRule Touch2 {
     X, Y
     { (articulated X) (Object Y) (touch X Y)! }
     { (touch X Y) (articulated Y) }
}

noDecisionRule CylinOnBoard {
     X, Y
     { (Cylin X) (Board Y) (loaded X)! (Gsupport Y X)! }
     { (loaded Y) (Gsupport Y X) }
}

noDecisionRule BoardOnCylin {
     X, Y
     { (Board X) (Cylin Y) (loaded X)! (loaded Y)! }
     { (loaded Y) (Gsupport Y X) }
}

noDecisionRule BoardOn2Cylin {
     X, Y, Z
     { (Board X) (Cylin Y) (Cylin Z) (loaded X)! (loaded Y)! (loaded Z)! }
     { (loaded Y) (loaded Z) (Gsupport Y X) (Gsupport Z X) }
}

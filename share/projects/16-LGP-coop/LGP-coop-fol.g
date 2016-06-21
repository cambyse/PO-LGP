Include = '../../../share/data/keywords.g'
Include = 'LGP-coop-kin.g'

FOL_World{
  hasWait=false
}

## activities
grasping
placing
attaching

graspingScrew
placingScrew

## basic predicates
agent
object

busy     # involved in an ongoing (durative) activity
free     # agent hand is free
held     # object is held by an agent
grasped  # agent X holds/has grasped object Y
hasScrew # agent X holds a screw (screws are not objects/constrants, just a predicate of having a screw...)
fixed    # object X and Y are fixed together
never    # (for debugging)

## constants
#A1
#A2
#A3
#Handle
#Long1
#Long2

## initial state
START_STATE {
#(agent A1) (free A1)
#(agent A2) (free A2)
#(agent A3) (free A3)
#(object Handle)
#(object Long1)
#(object Long2)
}

### RULES

#termination rule
Rule {
  X
  { (fixed) }
  { (QUIT) }
}

DecisionRule activate_grasping {
  X, Y
  { (grasping X Y)! (agent X) (object Y) (free X) (held Y)! (busy X)! (busy Y)! }
  { (grasping X Y)=5.0 (busy X) (busy Y) }
}

Rule {
  X, Y
  { (Terminate grasping X Y) }
  { (Terminate grasping X Y)! (grasping X Y)! (grasped X Y) (free X)! (held Y) (busy X)! (busy Y)! }
  { (Terminate grasping X Y)! (grasping X Y)! (busy X)! (busy Y)! } # failure
  p=[.9 0.1]
}

DecisionRule activate_placing {
  X, Y, Z,
  { (placing X Y Z)! (grasped X Y) (busy X)! (busy Y)! (table Z) }
  { (placing X Y Z)=1.0 (busy X) (busy Y) }
}

Rule {
  X, Y, Z,
  { (Terminate placing X Y Z) }
  { (Terminate placing X Y Z)! (placing X Y)! (grasped X Y)! (free X) (held Y)! (busy X)! (busy Y)! }
}


DecisionRule activate_graspingScrew {
  X
  { (graspingScrew X)! (agent X) (free X) (busy X)! }
  { (graspingScrew X)=1.0 (busy X) }
}

Rule {
  X
  { (Terminate graspingScrew X) }
  { (Terminate graspingScrew X)! (graspingScrew X)! (free X)! (hasScrew X) (busy X)! }
}

#that's never a good action -> for simplicity remove
#DecisionRule activate_placingScrew {
#  X
#  { (never) (placingScrew X)! (agent X) (hasScrew X) (busy X)! }
#  { (placingScrew X)=1.0 (busy X) }
#}
#
#Term (Terminate placingScrew) {
#  X
#  { (placingScrew X)! (hasScrew X)! (free X) (busy X)! }
#}

DecisionRule activate_attaching {
  X, Y, Z
  { (attaching X Y Z)! (object X) (object Y) (agent Z) (fixed X Y)! (held X) (held Y) (hasScrew Z) (busy X)! (busy Y)! (busy Z)! }
  { (attaching X Y Z)=3.0 (busy X) (busy Y) (busy Z) }
}

Rule {
  X, Y, Z
  { (Terminate attaching X Y Z) }
  { (Terminate attaching X Y Z)! (attaching X Y Z)! (fixed X Y) (fixed Y X) (hasScrew Z)! (free Z) (busy X)! (busy Y)! (busy Z)! } #NOTE: fixed needs to be symmetric: both predicates are added!
}


### Reward

REWARD {
  tree{
    leaf{ X, { (fixed X) }, r=10. }
#    leaf{ X, Y, { (fixed Handle X) (fixed Handle Y) }, r=100. }
    leaf{ X, Y, Z, { (activate_attaching X Y Z) }, r=10. }
    weight=1.
  }
}

# =============================================================================

PoseProblemRule {
     Hand, Obj
     { (grasping Hand Obj) }
     {
#       (EqualZero GJK Hand Obj){ scale=100 } #this describes touch, nice, but not consistent with others
       (MinSumOfSqr posDiff Hand Obj){ time=[1 1] scale=1e3 }
#       (MinSumOfSqr quatDiff Hand Obj){ time=[1 1] scale=1e3 }
       (MakeJoint delete Obj)
       (MakeJoint rigidZero Hand Obj)

#       (MinSumOfSqr posDiff endeffWorkspace Obj){ time=[1 1] scale=1e1 }
#       (LowerEqualZero limitIneq){ scale=1e0 time=[0 1] }
#       (LowerEqualZero collisionExcept Hand Obj){ margin=.05 time=[0 1] }
     }
}

PoseProblemRule {
     Hand, Obj, Onto
     { (placing Hand Obj Onto) }
     { (EqualZero GJK Obj Onto){ target=[0 0 .05] scale=100 }
#       (MinSumOfSqr posDiff Obj Onto){ target=[0 0 .5] scale=10 }
       (MakeJoint delete Hand Obj)
       (MakeJoint transXYPhiAtFrom Onto Obj)
       (MinSumOfSqr vec Obj){ vec1=[0 0 1] target=[0 0 1] scale=100}

#       (MinSumOfSqr posDiff endeffWorkspace Obj){ time=[1 1] scale=1e1 }
#       (LowerEqualZero limitIneq){ scale=1e0 time=[0 1] }
#       (LowerEqualZero collisionExcept Hand Obj){ margin=.05 time=[0 1] }
     }
}

# =============================================================================

#SeqProblemRule {
#  Hand, Obj
#  { (grasping Hand Obj) }
#  { (MinSumOfSqr qItself){ order=1 time=[0.1 1] scale=1e0 }
##    (MinSumOfSqr qZeroVels){ order=1 time=[0 1] scale=1e3 }
#    (MinSumOfSqr posDiff Hand Obj){ time=[1 1] scale=1e3 }
#    (MinSumOfSqr quatDiff Hand Obj){ time=[1 1] scale=1e3 }
#    (MakeJoint delete Obj){ time=1 }
##    (MakeJoint rigidZero Hand Obj){ time=1 }

#    (MinSumOfSqr posDiff endeffWorkspace Obj){ time=[1 1] scale=1e1 }
#    (LowerEqualZero limitIneq){ scale=1e0 time=[0 1] }
#    (LowerEqualZero collisionExcept Hand Obj){ margin=.05 time=[0 1] }
#  }
#}

#SeqProblemRule {
#  Hand, Obj, Onto
#  { (placing Hand Obj Onto) }
#  {
##    (MakeJoint delete Hand Obj){ time=0 }
#    (MakeJoint transXYPhiZero Onto Obj){ time=0 from=<T t(0 0 .3)> }
#    (MinSumOfSqr qItself){ order=1 time=[0.1 1] scale=1e0 }
#    (MinSumOfSqr qZeroVels){ order=1  time=[0 1] scale=1e3 }
#    (MinSumOfSqr posDiff Hand Obj){ time=[1 1] scale=1e3 }
#    (MinSumOfSqr quatDiff Hand Obj){ time=[1 1] scale=1e3 }
##    (EqualZero GJK Obj Onto){ target=[0 0 .05] scale=100 }
##    (MinSumOfSqr posDiff Obj Onto){ time=[1 1] target=[0 0 .2] scale=1e-1 } #1/2 metre above the thing
##    (MinSumOfSqr vec Obj){ time=[1 1] vec1=[0 0 1] target=[0 0 1] scale=100} #upright

#    (MinSumOfSqr posDiff endeffWorkspace Obj){ time=[1 1] scale=1e1 }
#    (LowerEqualZero limitIneq){ scale=1e0 time=[0 1] }
#    (LowerEqualZero collisionExcept Hand Obj){ margin=.05 time=[0 1] }

#  }
#}

## =============================================================================

#PathProblemRule {
#  Hand, Obj
#  { (grasping Hand Obj) }
#  { _MinSumOfSqr_qItself(MinSumOfSqr qItself){ time=[0 1] order=2 Hmetric=1e-1 } #transitions
#    #(EqualZero GJK Hand Obj){ time=[1 1] scale=100 } #touch is not necessary
#    _MinSumOfSqr_qItself_vel(MinSumOfSqr qItself){ order=1 time=[0.98 1] scale=1e1 } #slow down
#    (MinSumOfSqr posDiff Hand Obj){ time=[.98 1] scale=1e3 }
#    (MinSumOfSqr quatDiff Hand Obj){ time=[.98 1] scale=1e3 }

#    (MakeJoint delete Obj){ time=1 }
#    (MakeJoint rigidZero Hand Obj){ time=1 }
#  }
#}

#PathProblemRule {
#  Hand, Obj, Onto
#  { (placing Hand Obj Onto) }
#  { (MinSumOfSqr qItself){ time=[0 1] order=2 Hmetric=1e-1 } #transitions
#  #    (EqualZero GJK Obj Onto){ time=[1 1] target=[0 0 .05] scale=100 } #touch
#    (MinSumOfSqr pos Obj){ order=1 scale=1e-1 time=[0 0.15] target=[0 0 .1] } # move up
#    IhaveADifferentName(MinSumOfSqr qItself){ order=1 time=[0.98 1] scale=1e1 } #slow down
#    (MinSumOfSqr posDiff Obj Onto){ time=[1 1] target=[0 0 .2] scale=1000 } #1/2 metre above the thing
#    (MinSumOfSqr vec Obj){ time=[1 1] vec1=[0 0 1] target=[0 0 1] scale=100} #upright

#    (MakeJoint delete Hand Obj){ time=1 }
#    (MakeJoint rigidAtTo Onto Obj){ time=1 }
#  }
#}

## =============================================================================

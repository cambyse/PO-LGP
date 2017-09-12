Include = '../../data/keywords.g'
#Include = 'kin-stickHandover.g'

FOL_World{
  hasWait=false
  gamma = 1.
  stepCost = 1.
  timeCost = 0.
}

## activities
grasping
placing
handing
attaching
pushing

## basic predicates
agent
object
table
attachable
pusher
partOf
world

busy     # involved in an ongoing (durative) activity
free     # agent hand is free
held     # object is held by an agent
grasped  # agent X holds/has grasped object Y
placed  # agent X holds/has grasped object Y
attached
hasScrew # agent X holds a screw (screws are not objects/constrants, just a predicate of having a screw...)
fixed    # object X and Y are fixed together
never    # (for debugging)

## constants (added by the code)

## initial state (generated by the code)
START_STATE {}

### RULES

#####################################################################

#termination rule
#Rule {
#  { (grasped baxterR obj1) }
#  { (QUIT) }
#}

### Reward
REWARD {
#  tree{
#    leaf{ { (grasped handR screwdriverHandle) }, r=10. }
#    weight=1.
#  }
}

#####################################################################

DecisionRule grasp {
  X, Y
  { (INFEASIBLE grasp X Y)! (agent X) (object Y) (free X) (held Y)! }
  { (grasped X Y) (held Y) (free X)! komoGrasp(X Y)=1. }
}

#####################################################################

DecisionRule handover {
  X, Y, Z
  { (INFEASIBLE handover X Y Z)! (grasped X Y) (agent X) (agent Z) (object Y) (free Z) }
  { (grasped X Y)! (grasped Z Y) (free X) (free Z)! komoHandover(X Y Z)=1. }
}

#####################################################################

DecisionRule place {
  X, Y, Z,
  { (agent X) (object Y) (grasped X Y) (table Z) }
  { (placed Y Z) (grasped X Y)! (free X) (held Y)! komoPlace(X Y Z)=1. (INFEASIBLE grasp ANY Y)! block(INFEASIBLE grasp ANY Y)}
}

#####################################################################

DecisionRule place2 {
  Y, Z,
  { (held Z) (object Y) (table Z) }
  { (placed Y Z) komoPlace(Y Z)=1. (INFEASIBLE grasp ANY Y)! block(INFEASIBLE grasp ANY Y)}
}

#####################################################################

DecisionRule push {
  W, X, Y, Z,
  { (held W) (pusher X) (partOf X W) (object Y) (table Z) (held Y)! }
  { komoPush(X Y Z)=1. (INFEASIBLE grasp ANY Y)! block(INFEASIBLE grasp ANY Y) }
}

#####################################################################

DecisionRule slide {
  X, Y, Z,
  { (agent X) (object Y) (table Z) (held Y)! }
  { komoSlide(X Y Z)=1. (free X)! (held Y) (grasped X Y) (INFEASIBLE grasp ANY Y)! block(INFEASIBLE grasp ANY Y) }
}

#####################################################################

DecisionRule push2 {
  X, Y, Z,
  { (held X) (pusher X) (object Y) (table Z) (held Y)! }
  { komoPush(X Y Z)=1. (grasped X Y) (INFEASIBLE grasp ANY Y)! block(INFEASIBLE grasp ANY Y) }
}

#####################################################################

DecisionRule drop {
  OBJ, FROM, TO,
  { (held OBJ)! (object OBJ) (table FROM) (table TO) }
  { (grasped world OBJ) komoDrop(OBJ FROM TO)=1. (INFEASIBLE grasp ANY OBJ)! block(INFEASIBLE grasp ANY OBJ) }
}


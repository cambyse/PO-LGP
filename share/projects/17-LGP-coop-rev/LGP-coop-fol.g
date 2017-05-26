Include = '../../data/keywords.g'
Include = 'LGP-coop-kin.g'

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

graspingScrew
placingScrew

## basic predicates
agent
object
table
attachable

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

#termination rule
Rule {
  { (grasped handL screwdriverHandle) (grasped handR screwbox) }
  { (QUIT) }
}

### Reward
REWARD {
#  tree{
#    leaf{ { (grasped handR screwdriverHandle) }, r=10. }
#    weight=1.
#  }
}

DecisionRule activate_grasping {
  X, Y
  { (grasping X Y)! (INFEASIBLE activate_grasping X Y)! (agent X) (object Y) (free X) (held Y)! (busy X)! (busy Y)! }
  { (grasping X Y)=1.0 (busy X) (busy Y) komoGrasp(X Y)=1. }
}

## that directly terminates!!
Rule {
  X, Y
  { (grasping X Y) }
  { (Terminate grasping X Y) }
}

Rule {
  X, Y
  { (Terminate grasping X Y) }
  { (Terminate grasping X Y)! (grasping X Y)! (grasped X Y) (free X)! (held Y) (busy X)! (busy Y)! }
#  { (Terminate grasping X Y)! (grasping X Y)! (busy X)! (busy Y)! } # failure
#  p=[.9 0.1]
}

DecisionRule activate_handing {
  X, Y, Z
  { (handing X Y Z)! (INFEASIBLE) (INFEASIBLE activate_handing X Y Z)! (grasped X Y) (busy X)! (agent X) (agent Z) (object Y) (free Z) (busy Z)! }
  { (handing X Y Z)=1.0 (busy Y) (busy Z) komoHandover(X Y Z)=1. }
}

## that directly terminates!!
Rule {
  X, Y, Z
  { (handing X Y Z) }
  { (Terminate handing X Y Z) }
}

Rule {
  X, Y, Z
  { (Terminate handing X Y Z) }
  { (Terminate handing X Y Z)! (handing X Y Z)! (grasped X Y)! (free Z)! (free X) (grasped Z Y) (held Y) (busy X)! (busy Y)! (busy Z)!}
}

DecisionRule activate_placing {
  X, Y, Z,
  { (placing X Y Z)! (grasped X Y) (busy X)! (busy Y)! (table Z) }
  { (placing X Y Z)=1.0 (busy X) (busy Y) komoPlace(X Y Z)=1. (INFEASIBLE activate_grasping ANY Y)! block(INFEASIBLE activate_grasping ANY Y)}
}

## that directly terminates!!
Rule {
  X, Y, Z,
  { (placing X Y Z) }
  { (Terminate placing X Y Z) }
}

Rule {
  X, Y, Z,
  { (Terminate placing X Y Z) }
  { (Terminate placing X Y Z)! (placing X Y Z)! (placed Y Z) (grasped X Y)! (free X) (held Y)! (busy X)! (busy Y)! }
}


DecisionRule activate_attaching {
  X, Y, Z,
  { (grasped X Z) (object Y) (object Z) (attachable Y Z) }
  { (attached Y Z) (grasped X Z)! (free X) (held Z)! komoAttach(X Y Z)=1. }
}


#DecisionRule activate_graspingScrew {
#  X
#  { (graspingScrew X)! (agent X) (free X) (busy X)! }
#  { (graspingScrew X)=1.0 (busy X) }
#}

#Rule {
#  X
#  { (Terminate graspingScrew X) }
#  { (Terminate graspingScrew X)! (graspingScrew X)! (free X)! (hasScrew X) (busy X)! }
#}

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

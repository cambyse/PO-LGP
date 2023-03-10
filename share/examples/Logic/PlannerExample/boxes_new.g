FOL_World{
  maxHorizon=20
}

## Syntactic keywords
QUIT
WAIT
Terminate

## activities
grasping
graspingScrew
releasing
releasingScrew
fixing

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
A1
A2
A3
Handle
Long1
Long2

## initial state
START_STATE {
(agent A1) (free A1)
(agent A2) (free A2)
(agent A3) (free A3)
(object Handle)
(object Long1)
(object Long2)
}

### RULES

#termination rule
Rule {
  { (fixed Handle Long1) (fixed Handle Long2) }
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

DecisionRule activate_releasing {
  X, Y
  { (releasing X Y)! (grasped X Y) (busy X)! (busy Y)! }
  { (releasing X Y)=1.0 (busy X) (busy Y) }
}

Rule {
  X, Y
  { (Terminate releasing X Y) }
  { (Terminate releasing X Y)! (releasing X Y)! (grasped X Y)! (free X) (held Y)! (busy X)! (busy Y)! }
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
#DecisionRule activate_releasingScrew {
#  X
#  { (never) (releasingScrew X)! (agent X) (hasScrew X) (busy X)! }
#  { (releasingScrew X)=1.0 (busy X) }
#}
#
#Term (Terminate releasingScrew) {
#  X
#  { (releasingScrew X)! (hasScrew X)! (free X) (busy X)! }
#}

DecisionRule activate_fixing {
  X, Y, Z
  { (fixing X Y Z)! (object X) (object Y) (agent Z) (fixed X Y)! (held X) (held Y) (hasScrew Z) (busy X)! (busy Y)! (busy Z)! }
  { (fixing X Y Z)=3.0 (busy X) (busy Y) (busy Z) }
}

Rule {
  X, Y, Z
  { (Terminate fixing X Y Z) }
  { (Terminate fixing X Y Z)! (fixing X Y Z)! (fixed X Y) (fixed Y X) (hasScrew Z)! (free Z) (busy X)! (busy Y)! (busy Z)! } #NOTE: fixed needs to be symmetric: both predicates are added!
}


### Reward

REWARD {
  tree1{
    leaf{ X, Y, { (fixed Handle X) (fixed Handle Y) }, r=100. }
    weight=1.
  }
  tree2{
    leaf{ X, Y, Z, { decision(activate_fixing X Y Z) }, r=10. }
    weight=1.
  }
}

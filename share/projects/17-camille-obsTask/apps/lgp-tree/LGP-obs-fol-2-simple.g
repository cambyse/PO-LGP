Include = 'data/keywords.g'
Include = 'LGP-obs-kin-2.g'

FOL_World{
  hasWait=false
  gamma = 1.
  stepCost = 1.
  timeCost = 0.
}

## activities
getting_sight
observing

## basic predicates
agent
location
object

in_sight   # location X is visible
view_taken # object recognition has been triggered for checking if the object X is at location Y
viewed	   # object X has been viewed at location Y
at         # object X is at location Y

## constants
target_location_1
target_location_2

## initial state
START_STATE { (object target) (location target_location_1) (location target_location_2)}
BELIEF_START_STATE{ (at target target_location_1)=0.2 (at target target_location_1)=0.8 }
### RULES

#termination rule
Rule {
  { (view_taken target target_location_1) (view_taken target target_location_2)} # 
  { (QUIT) }
}

### Reward
REWARD {
#  tree{
#    leaf{ { (grasped handR screwdriverHandle) }, r=10. }
#    weight=1.
#  }
}

### Activities definitions
# Get sight
DecisionRule get_sight {
  X
  { (INFEASIBLE get_sight X)! (location X) }
  { (in_sight X)  komoGetSight(X)=1. }
}

# Take a view
DecisionRule take_view {
  X, Y
  { (object X) (location Y) (in_sight Y)}
  { (view_taken X Y) komoTakeView(X Y) }
}

# Rule {
  X, Y
  { (object X) (location Y) (view_taken X Y) }
  { (viewed X Y) }
}

##DecisionRule activate_grasping {
##  X, Y
##  { (grasping X Y)! (INFEASIBLE activate_grasping X Y)! (agent X) (object Y) (free X) (held Y)! (busy X)! (busy Y)! }
##  { (grasping X Y)=1.0 (busy X) (busy Y) komoGrasp(X Y)=1. }
##}

## that directly terminates!!
##Rule {
##  X, Y
##  { (grasping X Y) }
##  { (Terminate grasping X Y) }
##}

##Rule {
##  X, Y
##  { (Terminate grasping X Y) }
##  { (Terminate grasping X Y)! (grasping X Y)! (grasped X Y) (free X)! (held Y) (busy X)! (busy Y)! }
#  { (Terminate grasping X Y)! (grasping X Y)! (busy X)! (busy Y)! } # failure
#  p=[.9 0.1]
##}

##DecisionRule activate_handing {
## X, Y, Z
##  { (handing X Y Z)! (INFEASIBLE) (INFEASIBLE activate_handing X Y Z)! (grasped X Y) (busy X)! (agent X) (agent Z) (object Y) (free Z) ##(busy Z)! }
##  { (handing X Y Z)=1.0 (busy Y) (busy Z) komoHandover(X Y Z)=1. }
##}

## that directly terminates!!
##Rule {
##  X, Y, Z
##  { (handing X Y Z) }
##  { (Terminate handing X Y Z) }
##}

##Rule {
##  X, Y, Z
##  { (Terminate handing X Y Z) }
##  { (Terminate handing X Y Z)! (handing X Y Z)! (grasped X Y)! (free Z)! (free X) (grasped Z Y) (held Y) (busy X)! (busy Y)! (busy Z)!}
##}

##DecisionRule activate_placing {
##  X, Y, Z,
##  { (placing X Y Z)! (grasped X Y) (busy X)! (busy Y)! (table Z) }
##  { (placing X Y Z)=1.0 (busy X) (busy Y) komoPlace(X Y Z)=1. (INFEASIBLE activate_grasping ANY Y)! block(INFEASIBLE activate_grasping ANY ##Y)}
##}

## that directly terminates!!
##Rule {
##  X, Y, Z,
##  { (placing X Y Z) }
##  { (Terminate placing X Y Z) }
##}

##Rule {
##  X, Y, Z,
##  { (Terminate placing X Y Z) }
##  { (Terminate placing X Y Z)! (placing X Y Z)! (placed Y Z) (grasped X Y)! (free X) (held Y)! (busy X)! (busy Y)! }
##}


##DecisionRule activate_attaching {
##  X, Y, Z,
##  { (grasped X Z) (object Y) (object Z) (attachable Y Z) }
##  { (attached Y Z) (grasped X Z)! (free X) (held Z)! komoAttach(X Y Z)=1. }
##}


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

##DecisionRule activate_attaching {
##  X, Y, Z
##  { (attaching X Y Z)! (object X) (object Y) (agent Z) (fixed X Y)! (held X) (held Y) (hasScrew Z) (busy X)! (busy Y)! (busy Z)! }
##  { (attaching X Y Z)=3.0 (busy X) (busy Y) (busy Z) }
##}

##Rule {
##  X, Y, Z
##  { (Terminate attaching X Y Z) }
##  { (Terminate attaching X Y Z)! (attaching X Y Z)! (fixed X Y) (fixed Y X) (hasScrew Z)! (free Z) (busy X)! (busy Y)! (busy Z)! } #NOTE: ##fixed needs to be symmetric: both predicates are added!
##}

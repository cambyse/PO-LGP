Terminate
QUIT
WAIT

# activities
pick
give
hold
go_home_right
go_home_left
screw
position

# predicates
attached
positioned
in_human_ws
picked
at_home
busy # executing some activity
free 

occupied_slot
attached_slot
no_slot
one_slot
two_slot

holding_position
object
object1
object2
object3

# constants
right
left
human
/toolbox/handle
/toolbox/side_right
/toolbox/side_left
/toolbox/side_front
/toolbox/side_back
0
1

### RULES

DecisionRule activate_pick {
  Y 
  { (busy left)! (object Y) (picked Y)! (in_human_ws Y)! (free left) }
  { (pick Y)=1 (busy left) (at_home left)! }
}

Rule {
  Y
  { (Terminate pick Y) }
  { (Terminate pick Y)! (pick Y)! (busy left)! (free left)! (picked Y) }
}

DecisionRule activate_give {
  Y
  { (busy left)! (picked Y) }
  { (give Y)=1 (busy left) (at_home left)! }
}

Rule {
  Y
  { (Terminate give Y) }
  { (Terminate give Y)! (give Y)! (busy left)! (picked Y)! (in_human_ws Y) (free left) (at_home left)!}
}

DecisionRule activate_hold {
  X, Z
  { (in_human_ws X) (busy right)! (holding_position Z) (object3 X)! (attached_slot X Z)! }
  { (hold X Z)=2 (busy right) (at_home right)! }
}
 
Rule {
  X, Z
  { (Terminate hold X Z) }
  { (Terminate hold X Z)! (hold X Z)! (busy right)! }
}

DecisionRule activate_screw {
  X, Y, Z
  { (screw X Y Z)! (busy human)! (positioned X Y Z) (hold X Z)=2 (attached X Y Z)! }
  { (screw X Y Z)=1 (busy human) }
}

Rule {
  X, Y, Z
  { (Terminate screw X Y Z) }
  { (Terminate screw X Y Z)! (screw X Y Z)! (busy human)! (attached X Y Z) (attached_slot X Z) }
}

DecisionRule activate_position {
  X, Y, Z
  { (positioned X Y 0)! (positioned X Y 1)! (busy human)! (occupied_slot X Z)! (no_slot Y)! (in_human_ws Y) (object1 X) (object2 Y) (holding_position Z) (in_human_ws X) (in_human_ws Y) }
  { (position X Y Z)=1 (busy human) }
}

DecisionRule activate_position {
  X, Y, Z
  { (positioned X Y 0)! (positioned X Y 1)! (positioned /toolbox/side_left Y Z)! (positioned /toolbox/side_right Y Z)! (busy human)! (occupied_slot X Z)! (no_slot Y)! (in_human_ws Y) (object2 X) (object3 Y) (holding_position Z)  (in_human_ws X) (in_human_ws Y) }
  { (position X Y Z)=1 (busy human) }
}

Rule {
  X, Y, Z
  { (Terminate position X Y Z) (one_slot Y) }
  { (Terminate position X Y Z)! (position X Y Z)! (busy human)! (positioned X Y Z) (occupied_slot X Z) (one_slot Y)! (no_slot Y) }
}

Rule {
  X, Y, Z
  { (Terminate position X Y Z) (two_slot Y) }
  { (Terminate position X Y Z)! (position X Y Z)! (busy human)! (positioned X Y Z) (occupied_slot X Z) (two_slot Y)! (one_slot Y) }
}

DecisionRule activate_go_home_right {
  { (busy right)! (at_home right)!}
  { (go_home_right)=1 (busy right)}
}

Rule {
  { (Terminate go_home_right) }
  { (Terminate go_home_right)! (go_home_right)! (busy right)! (at_home right) }
}

DecisionRule activate_go_home_left {
  { (busy left)! (at_home left)! (free left)}
  { (go_home_left)=1 (busy left)}
}

Rule {
  { (Terminate go_home_left) }
  { (Terminate go_home_left)! (go_home_left)! (busy left)! (at_home left) }
}

#World rules

Rule {
  W, X, Y, Z, A, B
  { (positioned X Y A) (positioned X Z B) (positioned W Y B) }
  { (positioned W Z A) (occupied_slot W A) }
}

Rule {
  X, Y, Z, A, B
  { (positioned /toolbox/handle X A) (positioned /toolbox/handle Y B) (positioned X Z A) }
  { (positioned Y Z B) (occupied_slot Y B) }
}

Rule {
  X, Y, Z, A, B
  { (positioned /toolbox/handle X A) (positioned /toolbox/handle Y B) (positioned X Z B) }
  { (positioned Y Z A) (occupied_slot Y A) }
}

Rule {
  X, Y, Z, A, B
  { (positioned Y Z B) (positioned /toolbox/handle Y B) (positioned X Z A) }
  { (positioned /toolbox/handle X A) (occupied_slot /toolbox/handle A) }
}

Rule {
  X, Y, Z, A, B
  { (positioned Y Z A) (positioned /toolbox/handle Y B) (positioned X Z B) }
  { (positioned /toolbox/handle X A) (occupied_slot /toolbox/handle A) }
}

REWARD {
	tree{
		#leaf{ X, Y, Z, {(activate_screw X Y Z)}, r=5.}
		leaf{ X, Y, Z, {(screw X Y Z)}, r=5.}
		leaf{ { aggregate{X, Y, Z, {(attached X Y Z)}, count=6} }, r=100. }
		weight=1.
	}
}

START_STATE {
 (at_home right)
 (object /toolbox/handle)
 (object /toolbox/side_front)
 (object /toolbox/side_back)
 (object /toolbox/side_left)
 (object /toolbox/side_right)
 (object1 /toolbox/handle)
 (object2 /toolbox/side_left)
 (object2 /toolbox/side_right)
 (object3 /toolbox/side_front)
 (object3 /toolbox/side_back)
 (holding_position 0)
 (holding_position 1)
 (no_slot /toolbox/handle)
 (one_slot /toolbox/side_left)
 (one_slot /toolbox/side_right)
 (two_slot /toolbox/side_front)
 (two_slot /toolbox/side_back)
 (free left)
 (at_home left)
}

#Goal state
Rule {
  { aggregate{X, Y, Z, {(attached X Y Z)}, count=6} }
  { (QUIT) }
}

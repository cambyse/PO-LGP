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
glued

Board
Cylin

#objects
table1
table2
#a
#b

START_STATE{
  (Board table1)
  (Board table2)
# (Object a) (Cylin a)
# (Object a) (Cylin b)
  (articulated eff) (free eff)
}

REWARD{}

DecisionRule Glue {
     X, Y
     { (articulated X) (Object Y) (free X) }
     { (touch X Y) (articulated Y) (glued X Y) (free X)! (free Y) }
}

DecisionRule Release {
     X, Z
     { (articulated X) (free X) (Object X) (Board Z) }
     { (articulated X)! (touch X Z) (Gsupport X Z) }
}

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

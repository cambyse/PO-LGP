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
  (Object table1) (Board table1)
  (Object table2) (Board table2)
# (Object a) (Cylin a)
# (Object a) (Cylin b)
  (Object eff) (articulated eff)
}

REWARD{}

DecisionRule Glue {
     X, Y
     { (articulated X) (Object Y) (articulated Y)! }
     { (touch X Y) (articulated Y) (glued X Y) }
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

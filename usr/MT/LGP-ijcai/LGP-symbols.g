Terminate
QUIT

#symbols
Object
Gsupport
loaded
free
hand
touch

Board
Cylin

#objects
table1
a
b

START_STATE{
  (Object table1)
  (Object a)
  (Object b)
  (Board table1)
  (Cylin a)  
  (Cylin b)  
  (free hand)
}

REWARD{}

DecisionRule TouchObject {
     X
     { (free hand) (Object X) }
     { (free hand)! (touch X) }
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

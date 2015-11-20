Include = '../../../share/data/keywords.g'

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

DecisionRule Pick {
     Hand, Obj
     { (articulated Hand) (Object Obj) (free Hand) }
     { (articulated Obj) (holding Hand Obj) (free Hand)! (free Obj) }
}

DecisionRule Release {
     Hand, Obj, Onto
     { (articulated Obj) (Object Obj) (holding Hand Obj) (Board Onto) }
     { (articulated Obj)! (free Hand) (holding Hand Obj)! }
}

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
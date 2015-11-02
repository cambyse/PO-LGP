Include = '../../../data/keywords.g'
Include = 'model.g'

repeats = 1

KOMO{
  T = 0
  duration = 5
}

(EqualZero posDiff endeff bar1){ vec1=[.1 0 0] time=[1 1] scale=100 }
(LowerEqualZero collisionIneq){ margin=0.05 }
(MakeJoint rigid endeff bar1)

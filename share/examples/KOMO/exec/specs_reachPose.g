Include = '../../../data/keywords.g'
Include = 'model.g'

repeats = 1

KOMO{
  T = 1
  duration = 5
  meldFixedJoints, makeConvexHulls, activateAllContacts
}

(EqualZero posDiff endeff bar1){ vec1=[.1 0 0] time=[1 1] scale=100 }
(LowerEqualZero collisionIneq){ margin=0.05 }
(MakeJoint rigidZero endeff bar1)

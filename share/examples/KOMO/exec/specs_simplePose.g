Include = '../../../data/keywords.g'
Include = '../easy/model.kvg'

KOMO{
  T = 0
  k_order = 0
  duration = 100
  activateAllContacts
}

(EqualZero posDiff endeff target)
(LowerEqualZero collisionIneq){ margin=0.05 scale=.1 }


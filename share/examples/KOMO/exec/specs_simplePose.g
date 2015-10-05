KOMO{
  model = '../easy/model.kvg'
  T = 0
  k_order = 0
  duration = 100
  activateAllContacts
}

Task finalHandPosition{
  map={ type=posDiff ref1=endeff ref2=target }
  time=[1 1]
  scale=1
  type=equal
}

Task collisions{
  map={ type=collisionIneq margin=0.05 }
  type=inequal # hard inequality constraint
  scale = .1
}


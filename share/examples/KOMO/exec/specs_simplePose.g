KOMO{
  model = '../easy/model.kvg'
  T = 1
  duration = 5
}

Task step{ #these are the 'motion costs', stepping from the initial pose to the final
  map={ type=qItself }
  order=1
  scale=1
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

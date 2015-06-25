KOMO{
  model = 'model.kvg'
  T = 100
  duration = 5
}

Task sqrAccelerations{
  map={ type=qItself }
  order=2    # accelerations (default is 0)
  time=[0 1] # from start to end (default is [0 1])
  type=sumOfSqr  # squared costs (default is 'sumOfSqr')
  scale=1    # factor of the map (default is [1])
  target=[0] # offset of the map (default is [0])
}

Task finalHandPosition{
  map={ type=posDiff ref1=endeff ref2=target vec1=[.15 0 0] }
  time=[1 1] # only final
  type=equal # hard equality constraint
}

Task finalHandVelocity{
  map={ type=pos ref1=endeff }
  order=1
  time=[1 1] # only final
#  type=equal # hard equality constraint
}

Task finalHandAlignment{
  map={ type=vecAlign ref1=endeff ref2=target vec1=[1 0 0] vec2=[1 0 0] }
  time=[1 1] # only final
  target = [1]
  type=equal # hard equality constraint
}

Task collisions{
  map={ type=collisionIneq margin=0.1 }
  type=inequal # hard inequality constraint
  scale = 0.1
}

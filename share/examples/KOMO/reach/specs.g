KOMO{
  model = 'model.g'
  T = 20
  duration = 5
  phases = 1
}

Task sqrAccelerations{
  map={ type=qItself Hmetric=1. }
  order=2    # accelerations (default is 0)
  time=[0 2] # from start to end (default is [0 1])
  type=sumOfSqr  # squared costs (default is 'sumOfSqr')
  scale=1    # factor of the map (default is [1])
  target=[0] # offset of the map (default is [0])
}

Task {
  map={ type=qItself }
  order=1
  time=[1 1] # only final
}

NoTask {
  map={ type=posDiff ref1=hook ref2=objs }
  time=[1 1] # only final
  target=[-.2 0 0]
}

Task hook{
  map={ type=GJK_vec ref1=hook ref2=objs }
  time=[1 1]
  scale=1e2
  target=[-.2 0 0]
  type=equal
}

NoTask coll{
  map={ type=GJK_vec ref1=hookcol ref2=objcol }
  time[0 1]
  scale=-1
  target=[-.1]
  type=inequal
}

NoTask collisions{
  map={ type=collisionIneq margin=0.08 }
  time=[0 1]
  scale=1
  type=inequal
}

NoTask collisions{
  map={ type=proxy margin=0.05 }
  time=[0 1]
  scale=1
#  type=inequal
}

KOMO{
  model = 'model.g'
  T = 100
  duration = 5
  phases = 2
}

Task sqrAccelerations{
  map={ type=qItself }
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
#  type=equal # hard equality constraint
}

Task {
  map={ type=qItself }
  order=1
  time=[2 2] # only final
#  type=equal # hard equality constraint
}

Task firstHandPosition{
  map={ type=posDiff ref1=graspRef ref2=handle }
  time=[1 1] # only final
#  type=equal # hard equality constraint
}

Task firstHandQuat{
  map={ type=quatDiff ref1=graspRef ref2=handle }
  time=[1 1] # only final
#  type=equal # hard equality constraint
}

Task secondHandPosition{
  map={ type=posDiff ref1=hook ref2=objs vec2=[0 0 0] }
  time=[2 2] # only final
#  type=equal # hard equality constraint
}

NoTask finalHandAlignment{
  map={ type=vecAlign ref1=endeff ref2=target vec1=[1 0 0] vec2=[1 0 0] }
  time=[1 1] # only final
  target = [1]
  type=equal # hard equality constraint
}

NTask collisions{
  map={ type=collisionIneq margin=0.05 }
  type=inequal # hard inequality constraint
  scale = 0.1
}

KinematicSwitch{
  type=addRigid
  timeOfApplication=1
  from=graspRef
  to=handle
}

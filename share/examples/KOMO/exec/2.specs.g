repeats = 1

KOMO{
  model = '1.pose.g'
  T = 30
  duration = 5
}

Task step{ #these are the 'motion costs', stepping from the initial pose to the final
  map={ type=qItself }
  order=1
  scale=1
}

Task finalHookPosition{
  map={ type=posDiff ref1=hook1 vec1=[0 .15 0] ref2=bar2 }
  time=[1 1]
  scale=100
  type=equal
}

Task midHandPosition{
  map={ type=posDiff ref1=endeff vec1=[0 0 1.5] }
  time=[.5 .5]
  scale=100
}

NoTask collisions{
  map={ type=collisionIneq margin=0.05 }
  type=inequal # hard inequality constraint
  scale = 1.
}

NoKinematicSwitch{
  type=addRigidRel
  timeOfApplication=1
  from=hook1
  to=bar2
}

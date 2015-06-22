Script {
  (FollowReferenceActivity wheels){ type=wheels, target=[0, .3, .2], PD=[.5, .9, .5, 10.]}
  (MyTask endeffR){ type=pos, ref2=base_footprint, target=[.2, -.5, 1.3], PD=[.5, .9, .5, 10.]}
  (MyTask endeffL){ type=pos, ref2=base_footprint, target=[.2, +.5, 1.3], PD=[.5, .9, .5, 10.]}
  { (conv FollowReferenceActivity wheels) }
  { (conv MyTask endeffL) }
  (MyTask endeffL)!, (MyTask endeffR)!, (conv MyTask endeffL)!, (conv MyTask endeffR)!, (FollowReferenceActivity wheels)!, (conv FollowReferenceActivity wheels)!

  (FollowReferenceActivity wheels){ type=wheels, target=[0, -.3, -.2], PD=[.5, .9, .5, 10.]}
  (MyTask endeffR){ type=pos, ref2=base_footprint, target=[.7, -.2, .7], PD=[.5, .9, .5, 10.]}
  (MyTask endeffL){ type=pos, ref2=base_footprint, target=[.7, +.2, .7], PD=[.5, .9, .5, 10.]}
  { (conv FollowReferenceActivity wheels) }
  { (conv MyTask endeffL) }
  (MyTask endeffL)!, (MyTask endeffR)!, (conv MyTask endeffL)!, (conv MyTask endeffR)!, (FollowReferenceActivity wheels)!, (conv FollowReferenceActivity wheels)!

  (FollowReferenceActivity wheels){ type=wheels, target=[0, .3, .2], PD=[.5, .9, .5, 10.]}
  (MyTask endeffR){ type=pos, ref2=base_footprint, target=[.2, -.5, 1.3], PD=[.5, .9, .5, 10.]}
  (MyTask endeffL){ type=pos, ref2=base_footprint, target=[.2, +.5, 1.3], PD=[.5, .9, .5, 10.]}
  { (conv MyTask endeffL) }
  (MyTask endeffL)!, (MyTask endeffR)!, (conv MyTask endeffL)!, (conv MyTask endeffR)!, (FollowReferenceActivity wheels)!, (conv FollowReferenceActivity wheels)!

  (FollowReferenceActivity wheels){ type=wheels, target=[0, 0, 0], PD=[.5, .9, .5, 10.]}
  (HomingActivity)
  { (conv HomingActivity) }
  { (conv FollowReferenceActivity wheels) }
}

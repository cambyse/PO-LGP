### objs

#vehicles
body car_ego { type=9 rel=<T t(0 0.0 0.05)> size=[.2 .1 .1 .02]  color=[0 1 1] contact }

#road
body ground{ type=9, X=<T t(0.0 -0.10 0.0)>, size=[5. 1.0 .04 .01], color=[.3 .3 .3] }

## Joints
joint (ground car_ego)         { from=<T t( -2.0   0.01 .05 ) t(0 0 0)> to=<T > type=8 agent=true }

#BELIEF_START_STATE { 
#{
#}
#{
#joint (lane_2 car_op)          { from=<T t( 0.9      0  .05 ) t(0 0 0)> to=<T >  type=3 }
#}
#}

#Include = 'arm7.kvg'
Include = '../../../data/pr2_model/pr2_model.ors'

body table{ type=0, X=<T t(1.2 0 .6)>, size=[1. 1. .04 .0], color=[.8 .5 .3] fixed,  }
shape leg1(table) { rel=<T t(-.3 -.3 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg2(table) { rel=<T t(-.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg3(table) { rel=<T t(.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg4(table) { rel=<T t(.3 -.3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }

#body slider { type=ST_box size=[.2 .1 .05 0] color=[0 0 0] }
#joint sliderJoint(table slider){ type=JT_hingeZ  ctrl_H=1 }

body obj1 { size=[.1 .1 .2 .02] type=9 contact }
joint (table obj1) { from=<T t(.2 -.4 .02)> to=<T t(0 0 .1)> type=JT_rigid}

shape endeff(r_wrist_roll_link){ type=9 rel=<T t(.3 0 0)> size=[.05 .05 .05 .02] color=[1 1 0] contact }
shape target(table){ type=5 rel=<T t(0 .3 .15)> size=[.3 0 0 0] color=[1 1 0]}

### tools

body hook { X=<T t(.2 0 .64)> }
shape (hook) { rel=<T t(-.2 0 0)> type=0 size=[.4 .025 .04 0] color=[1 0 0] contact }
shape (hook) { rel=<T t(0 0 0)> type=0 size=[.025 .2 .04 0] color=[1 0 0] contact }
shape hook_eff(hook) { rel=<T t(0 .08 0)> type=0 size=[.026.04 .04 0] color=[1 1 0] contact }
shape hook_handle(hook) { rel=<T t(-.3 0 0)> type=0 size=[.08 .026 .04 0] color=[1 1 0] }
joint (table hook) { from=<T t(0 0 .02)> to=<T t(0 0 .02)> type=JT_rigid}

body stick { X=<T t(.2 0 .64)> }
shape (stick) { rel=<T t(-.2 0 0)> type=0 size=[.4 .025 .04 0] color=[1 0 0] contact }
shape stick_handle(stick) { rel=<T t(-.3 0 0)> type=0 size=[.08 .026 .04 0] color=[1 1 0] }
joint (table stick) { from=<T t(0 .2 .02)> to=<T t(0 0 .02)> type=JT_rigid}

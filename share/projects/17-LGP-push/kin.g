#Include = 'arm7.kvg'
Include = '../../data/pr2_model/pr2_model.ors'
#Include = '../../data/baxter_model/baxter.ors'

body table1{ type=0, X=<T t(.7 0 .5)>, size=[1. 1. .04 .0], color=[.8 .5 .3] fixed,  }
shape leg1(table1) { rel=<T t(-.3 -.3 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg2(table1) { rel=<T t(-.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg3(table1) { rel=<T t(.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg4(table1) { rel=<T t(.3 -.3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }

body slider1a { type=ST_box size=[.2 .1 .05 0] color=[0 0 0] }
body slider1b { type=ST_box size=[.2 .1 .05 0] color=[1 0 0] }
joint (slider1a slider1b){ type=JT_transX }
shape slider1Eff (slider1b){ rel=<T t(.1 0 0)> type=5 size=[.1 .1 .1] color=[0 1 0] }

body obj1 { size=[.1 .1 .2 .02] type=9 contact }
joint (table1 obj1) { from=<T t(0 0 .02)> to=<T t(.2 -.4 .1)> type=JT_rigid }

shape target(table1){ type=5 rel=<T t(0 .3 .12)> size=[.3 0 0 0] color=[1 1 0]}

#shape baxterR (right_wrist){ rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> type=5 size=[.1 0 0 0] color=[1 1 0] }
#shape baxterL (left_wrist) { rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> type=5 size=[.1 0 0 0] color=[1 1 0] }

### tools

#body hook { X=<T t(.2 0 .64)> }
#shape (hook) { rel=<T t(-.2 0 0)> type=0 size=[.4 .025 .04 0] color=[1 0 0] contact }
#shape (hook) { rel=<T t(0 0 0)> type=0 size=[.025 .2 .04 0] color=[1 0 0] contact }
#shape hook_eff(hook) { rel=<T t(0 .08 0)> type=0 size=[.026.04 .04 0] color=[1 1 0] contact }
#shape hook_handle(hook) { rel=<T t(-.3 0 0)> type=0 size=[.08 .026 .04 0] color=[1 1 0] }
#joint (table1 hook) { from=<T t(0 0 .02)> to=<T t(0 0 .02)> type=JT_rigid}

body stick { X=<T t(.2 0 .64)> }
shape stick(stick) { rel=<T t(-.2 0 0)> type=9 size=[.4 .025 .04 .01] color=[1 0 0] contact }
shape stick_eff(stick) { rel=<T t(-.1 0 0)> type=0 size=[.08 .026 .04 0] color=[1 1 0] }
shape stick_handle(stick) { rel=<T t(-.3 0 0)> type=0 size=[.08 .026 .04 0] color=[1 1 0] }
joint (table1 stick) { from=<T t(0 .2 .02)> to=<T t(0 0 .02)> type=JT_rigid}

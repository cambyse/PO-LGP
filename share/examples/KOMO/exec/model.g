Include = '../../../data/pr2_model/pr2_model.ors'

Include = '../../../data/pr2_model/pr2_modify_fixHead.ors'
Include = '../../../data/pr2_model/pr2_modify_fixGrippers.ors'
Include = '../../../data/pr2_model/pr2_modify_fixLeft.ors'

#Include = '../../../examples/Ors/ors_editor/regal.ors'

shape endeff(r_wrist_roll_link){ rel=<T t(.2 0 0)> type=5 color=[1 0 0] size=[.1 0 0 0]}

### tables

body table1{ type=0, X=<T t(1.5 1. .6) d(60 0 0 1)>, size=[1. 1. .04 .0], color=[.8 .5 .3] fixed, contact }
shape leg1(table1) { rel=<T t(-.5 -.5 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg2(table1) { rel=<T t(-.5 .5 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg3(table1) { rel=<T t(.5 .5 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg4(table1) { rel=<T t(.5 -.5 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }

body table2{ type=0, X=<T t(1.5 -1. .6)>, size=[2. 2. .04 .0], color=[.8 .5 .3] fixed, contact }
shape leg1(table2) { rel=<T t(-.5 -.5 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg2(table2) { rel=<T t(-.5 .5 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg3(table2) { rel=<T t(.5 .5 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg4(table2) { rel=<T t(.5 -.5 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }

### tools

body tool1 { X=<T t(1.5 1. .64) d(240 0 0 1)> }
shape bar1(tool1) { rel=<T t(.4 0 0)> type=0 size=[.8 .04 .04 0] color=[1 0 0] _contact }
shape hook1(tool1) { rel=<T t(.02 .15 0)> type=0 size=[.04 .3 .04 0] color=[1 0 0] _contact }

body tool2 { X=<T t(2. -1. .64) d(-140 0 0 1)> }
shape bar2(tool2) { rel=<T t(.2 0 0)> type=0 size=[.4 .04 .04 0] color=[1 0 0] _contact }
shape hook2(tool2) { rel=<T t(.02 .1 0)> type=0 size=[.04 .2 .04 0] color=[1 0 0] _contact }

#shape hook(tool) { rel=<T t(.02 .2 0)> type=0 size=[.04 .04 .04 0] color=[1 1 0] contact }
#shape handle(tool) { rel=<T t(.5 0 0)> type=0 size=[.08 .04 .04 0] color=[1 1 0] }

#joint (graspRef tool){ B=<T t(-.5 0 0)> }

### tables

body table1{ type=0, X=<T t(.6 0 .6) d(60 0 0 1)>, size=[1. 1. .04 .0], color=[.8 .5 .3] fixed, _contact }
shape leg1(table1) { rel=<T t(-.5 -.5 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] _contact }
shape leg2(table1) { rel=<T t(-.5 .5 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] _contact }
shape leg3(table1) { rel=<T t(.5 .5 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] _contact }
shape leg4(table1) { rel=<T t(.5 -.5 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] _contact }

body table2{ type=0, X=<T t(-1. 0 .6)>, size=[1. 1. .04 .0], color=[.8 .5 .3] fixed, _contact }
shape leg1(table2) { rel=<T t(-.5 -.5 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] _contact }
shape leg2(table2) { rel=<T t(-.5 .5 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] _contact }
shape leg3(table2) { rel=<T t(.5 .5 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] _contact }
shape leg4(table2) { rel=<T t(.5 -.5 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] _contact }

### manipulator

body base { X=<T t(0 1 0)> size=[.1 .1 .1 .05] type=0 __contact, color=[0 0 0] }
body base2 { size=[.1 .1 .1 .05] type=1 __contact, color=[0 0 0] }
body arm1 { type=2 mass=1 size=[0.1 0.5 .3 .05] _contact, color=[.5 0 0]}
body arm2 { type=2 mass=1 size=[0.1 0.1 .3 .05] _contact, color=[.5 0 0] }
body eff { type=2 mass=1 size=[0.1 0.1 .3 .05] _contact, color=[.5 0 0] }
#body eff { type=0 mass=1 size=[0.1 0.1 .1 .05]  color=[.5 0 0] }

#body hand { size=[.05 .05 .2 .05] type=0 _contact, color=[1 0 0], ctrlable, canGrasp }
joint (base base2) { type=7 A=<T t(0 0 1.7)> H=1e3 }
#joint (base2 hand) { type=11 A=<T d(-160 0 1 0)>  }
joint (base2 arm1) { type=11 A=<T d(-50 1 0 0) d(-160 0 1 0)>  }
joint (arm1 arm2) { A=<T t(0 0 .15) d(90 0 0 1) d(-10 1 0 0)> B=<T t(0 0 .15) > }
joint (arm2 eff) { A=<T t(0 0 .15) d(90 0 0 1)  d(-10 1 0 0)> B=<T t(0 0 .15) > }

body graspRef { type=5 size=[.1 0 0 0] H=.01 }
joint graspJoint(eff graspRef){ A=<T t(0 0 .35)  d(-180 0 0 1) d(160 0 1 0) d(50 1 0 0) > type=11 }

### tools

body tool { X=<T t(.2 0 .64)> }
shape lbar(tool) { rel=<T t(.4 0 0)> type=0 size=[.8 .04 .04 0] color=[1 0 0] contact }
shape hookcol(tool) { rel=<T t(.02 .15 0)> type=0 size=[.04 .3 .04 0] color=[1 0 0] contact }

shape hook(tool) { rel=<T t(.02 .2 0)> type=0 size=[.04 .04 .04 0] color=[1 1 0] contact }
shape handle(tool) { rel=<T t(.5 0 0)> type=0 size=[.08 .04 .04 0] color=[1 1 0] }

joint (graspRef tool){ B=<T t(-.5 0 0)> }

### objs

body obj { X=<T t(-1. .3 .62)> }
shape objcol(obj) { rel=<T t(0 0 .1)> type=4 size=[0 0 .2 .05] color=[0 1 0] contact }
shape objs(obj) { rel=<T t(0 0 .1)> type=4 size=[0 0 .1 .05] color=[0 1 1] contact }

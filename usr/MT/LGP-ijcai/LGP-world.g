
body table1{ type=9, X=<T t(.6 0 .6) d(60 0 0 1)>, size=[1. 1. .02 .01], color=[.8 .5 .3] fixed, contact }
shape leg1(table1) { rel=<T t(-.3 -.3 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg2(table1) { rel=<T t(-.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg3(table1) { rel=<T t(.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg4(table1) { rel=<T t(.3 -.3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }

body table2{ type=9, X=<T t(-1. 0 .6)>, size=[1.7 2.5 .02 .01], color=[.8 .5 .3] fixed, contact }
shape leg1(table2) { rel=<T t(-.3 -.3 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg2(table2) { rel=<T t(-.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg3(table2) { rel=<T t(.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg4(table2) { rel=<T t(.3 -.3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }

body base { X=<T t(0 1 0)> size=[.1 .1 .1 .05] type=0 _contact, color=[0 0 0] }
body base2 { size=[.1 .1 .1 .05] type=1 _contact, color=[0 0 0] }
body arm1 { type=2 mass=1 size=[0.1 0.5 .3 .05] contact, color=[.5 0 0]}
body arm2 { type=2 mass=1 size=[0.1 0.1 .3 .05] contact, color=[.5 0 0] }
body eff { type=9 mass=1 size=[0. 0. .3 .05] contact, color=[.5 0 0] }
#body eff { type=0 mass=1 size=[0.1 0.1 .1 .05]  color=[.5 0 0] }

#body hand { size=[.05 .05 .2 .05] type=0 contact, color=[1 0 0], ctrlable, canGrasp }
joint (base base2) { type=7 A=<T t(0 0 1.7)> }
#joint (base2 hand) { type=11 A=<T d(-160 0 1 0)>  }
joint (base2 arm1) { type=11 A=<T d(-50 1 0 0) d(-160 0 1 0)>  }
joint (arm1 arm2) { A=<T t(0 0 .15) d(90 0 0 1) d(-10 1 0 0)> B=<T t(0 0 .15) > }
joint (arm2 eff) { A=<T t(0 0 .15) d(90 0 0 1)  d(-10 1 0 0)> B=<T t(0 0 .15) > }

#body graspRef { type=5 size=[.1 0 0 0] }
#joint graspJoint(eff graspRef){ A=<T t(0 0 .35)  d(-180 0 0 1) d(160 0 1 0) d(50 1 0 0) > type=11 }



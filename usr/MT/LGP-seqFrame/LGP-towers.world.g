
body table1{ type=9, X=<T t(1.5 0 .6) >, size=[1. 1. .02 .01], color=[.8 .5 .3] fixed, contact }
shape leg1(table1) { rel=<T t(-.3 -.3 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg2(table1) { rel=<T t(-.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg3(table1) { rel=<T t(.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg4(table1) { rel=<T t(.3 -.3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }

body table2{ type=9, X=<T t(-1. -1.5 .6)>, size=[2.5 1.5 .02 .01], color=[.8 .5 .3] fixed, contact }
shape leg1(table2) { rel=<T t(-.3 -.3 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg2(table2) { rel=<T t(-.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg3(table2) { rel=<T t(.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg4(table2) { rel=<T t(.3 -.3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }

body obj1 { size=[.1 .1 .2 .02] type=9 contact }
joint (table1 obj1) { from=<T t(-.1 .2 0)> to=<T t(0 0 .1)> type=10 }

#Include='floatingArm.ors'
Include='pr2.ors'



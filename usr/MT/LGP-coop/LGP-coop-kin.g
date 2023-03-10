Include = '../../../share/data/baxter_model/baxter.ors'

Include = '../../../share/data/man_model.ors'

Edit  waist { X=<T t(1.5 0. 1.) d(-90 0 0 1)> }

body table{ type=0, X=<T t(.7 0 .8)>, size=[1. 1. .04 .0], color=[.8 .5 .3] fixed, contact }
shape leg1(table) { rel=<T t(-.3 -.3 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg2(table) { rel=<T t(-.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg3(table) { rel=<T t(.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg4(table) { rel=<T t(.3 -.3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }

#body obj1 { size=[.06 .06 .1 .02] type=9 contact }
#joint (table obj1) { from=<T t(-.2 .2 0)> to=<T t(0 0 .1)> type=10 }
#shape shape1 (obj1) { type=5 rel=<T t(0 0 .1)>size=[.4 0 0 0] }

body Handle { type=0 size=[.03 .3 .15 .1] contact }
joint (table Handle) { from=<T t(-.2 .2 0)> to=<T t(0 0 .1)> type=10 }

body Long1 { type=0 size=[.03 .3 .1 .1] contact }
joint (table Long1) { from=<T t(.2 .2 0)> to=<T t(0 0 .1)> type=10 }

body Long2 { type=0 size=[.03 .3 .1 .1] contact }
joint (table Long2) { from=<T t(.2 -.2 0)> to=<T t(0 0 .1)> type=10 }

shape A1(left_wrist){ rel=<T t(.24 0 0)> type=5 color=[1 0 0] size=[.2 0 0 0]}
shape A2(right_wrist){ rel=<T t(.24 0 0)> type=5 color=[1 0 0] size=[.2 0 0 0]}
shape A3(handR){ type=5 color=[1 0 0] size=[.2 0 0 0]}

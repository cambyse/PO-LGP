#Include = 'arm7.kvg'
Include = '../../data/pr2_model/pr2_model.ors'

body table{ type=0, X=<T t(0 -1.2 .54)>, size=[1. 1. .04 .02], color=[.8 .5 .3] }
shape leg1(table) { rel=<T t(-.25 -.25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] }
shape leg2(table) { rel=<T t(-.25  .25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] }
shape leg3(table) { rel=<T t( .25  .25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] }
shape leg4(table) { rel=<T t( .25 -.25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] }

body table2{ type=0, X=<T t(.6 0 .53)>, size=[.6 .6 .04 .02], color=[.8 .5 .3] }

body obj1 { size=[.1 .1 .2 .02] type=0 contact }
joint (table obj1) { from=<T t(0 .3 .02)> to=<T t(0 0 .1)> type=JT_rigid }

body obj2 { size=[.1 .1 .2 .02] type=0 X=<T t(0 -1.2 .6) t(.3 .3 .02) t(0 0 .1) t(0 0 .01)> contact }
joint (table obj2) { from=<T t(.3 .3 .02)> to=<T t(0 0 .1)> type=JT_rigid }

shape objTarget(table2) { type=ST_marker rel=<T t(-.0 0 .03)> size=[.3 0 0 0] color=[1 .5 0] }

body ball { X=<T t(-1 -1.2 1.)>, size=[.1 .1 .2 .02] type=9 }
body ball2 { X=<T t(-1 -1.2 1.5)>, size=[.1 .1 .2 .02] type=9 }

body S1 { size=[.06 .06 .04 0] type=0 color=[.8 .8 .2] coloredBox percept }
joint (table2 S1) { from=<T t(0 -.15 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }

#body S2 { size=[.2 .06 .04 0] type=0 color=[.8 .8 .2] coloredBox }
#joint (table2 S2) { from=<T t(0 .15 .02)> to=<T t(0 0 .02)> type=JT_rigid }

#body S3 { size=[.2 .06 .04 0] type=0 color=[.8 .8 .2] coloredBox }
#joint (table2 S3) { from=<T t(0 .0 .02)> to=<T t(0 0 .02)> type=JT_rigid }

#body S4 { size=[.2 .06 .04 0] type=0 color=[.8 .8 .2] coloredBox }
#joint (table2 S4) { from=<T t(0 -.15 .02)> to=<T t(0 0 .02)> type=JT_rigid }

#body S5 { size=[.4 .025 .04 0] type=0 color=[.2 .8 .2] coloredBox }
#joint (table2 S5) { from=<T t(.2 0 .02)> to=<T d(90 0 0 1) t(0 0 .02)> type=JT_rigid }

#body S6 { size=[.4 .025 .04 0] type=0 color=[.2 .8 .2] coloredBox }
#joint (table2 S6) { from=<T t(-.2 0 .02)> to=<T d(90 0 0 1) t(0 0 .02)> type=JT_rigid }

#Include = 'arm7.kvg'
Include = '../../data/pr2_model/pr2_model.ors'

body table{ type=0, X=<T t(0 -1.2 .5)>, size=[1.2 1. .04 .02], color=[.8 .5 .3] contact }
shape leg1(table) { rel=<T t(-.25 -.25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] contact }
shape leg2(table) { rel=<T t(-.25  .25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] contact }
shape leg3(table) { rel=<T t( .25  .25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] contact }
shape leg4(table) { rel=<T t( .25 -.25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] contact }

body table2{ type=0, X=<T t(.6 0 .63)>, size=[.6 .6 .04 .02], color=[.8 .5 .3] }
#body table2{ type=0, X=<T t(.6 0 .52)>, size=[.6 .6 .04 .02], color=[.8 .5 .3] }
#body table2{ type=0, X=<T t(1.2 0 .5)>, size=[1. 1. .04 .02], color=[.8 .5 .3] contact }
#shape (table2) { rel=<T t(-.25 -.25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] contact }
#shape (table2) { rel=<T t(-.25  .25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] contact }
#shape (table2) { rel=<T t( .25  .25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] contact }
#shape (table2) { rel=<T t( .25 -.25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] contact }

body S1 { size=[.06 .06 .04 0] type=0 color=[.8 .8 .2] coloredBox percept }
joint (table2 S1) { from=<T t(0 -.15 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }

body S2 { size=[.06 .06 .04 0] type=0 color=[.8 .8 .2] coloredBox percept }
joint (table2 S2) { from=<T t(0 -.05 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }

body S3 { size=[.06 .06 .04 0] type=0 color=[.8 .8 .2] coloredBox percept }
joint (table2 S3) { from=<T t(0 .05 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }

body S4 { size=[.06 .06 .04 0] type=0 color=[.8 .8 .2] coloredBox percept }
joint (table2 S4) { from=<T t(0 .15 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }

body cube1 { size=[.1 .1 .1 .02] type=ST_ssBox contact }
joint (table cube1) { from=<T t(-.1 .4 .02)> to=<T t(0 0 .05)> type=JT_rigid }

body cube2 { size=[.1 .1 .1 .02] type=ST_ssBox contact }
joint (table cube2) { from=<T t(.1 .4 .02)> to=<T t(0 0 .05)> type=JT_rigid }

body cube3 { size=[.1 .1 .1 .02] type=ST_ssBox contact }
joint (table cube3) { from=<T t(-.1 .2 .02)> to=<T t(0 0 .05)> type=JT_rigid }

body cube4 { size=[.1 .1 .1 .02] type=ST_ssBox contact }
joint (table cube4) { from=<T t(.1 .2 .02)> to=<T t(0 0 .05)> type=JT_rigid }

body cube5 { size=[.1 .1 .1 .02] type=ST_ssBox contact }
joint (table cube5) { from=<T t(.3 .4 .02)> to=<T t(0 0 .05)> type=JT_rigid }

body cube6 { size=[.1 .1 .1 .02] type=ST_ssBox contact }
joint (table cube6) { from=<T t(.3 .2 .02)> to=<T t(0 0 .05)> type=JT_rigid }

body cube7 { size=[.1 .1 .1 .02] type=ST_ssBox contact }
joint (table cube7) { from=<T t(.5 .4 .02)> to=<T t(0 0 .05)> type=JT_rigid }

body cube8 { size=[.1 .1 .1 .02] type=ST_ssBox contact }
joint (table cube8) { from=<T t(.5 .2 .02)> to=<T t(0 0 .05)> type=JT_rigid }

body rect1 { size=[.1 .3 .1 .02] type=ST_ssBox contact }
joint (table rect1) { from=<T t(-.5 -.1 .02)> to=<T t(0 0 .05)> type=JT_rigid }

body rect2 { size=[.1 .3 .1 .02] type=ST_ssBox contact }
joint (table rect2) { from=<T t(-.3 -.1 .02)> to=<T t(0 0 .05)> type=JT_rigid }

body rect3 { size=[.1 .3 .1 .02] type=ST_ssBox contact }
joint (table rect3) { from=<T t(-.1 -.1 .02)> to=<T t(0 0 .05)> type=JT_rigid }

body rect4 { size=[.1 .3 .1 .02] type=ST_ssBox contact }
joint (table rect4) { from=<T t(.1 -.1 .02)> to=<T t(0 0 .05)> type=JT_rigid }

body rect5 { size=[.1 .3 .1 .02] type=ST_ssBox contact }
joint (table rect5) { from=<T t(.3 -.1 .02)> to=<T t(0 0 .05)> type=JT_rigid }

body rect6 { size=[.1 .3 .1 .02] type=ST_ssBox contact }
joint (table rect6) { from=<T t(.5 -.1 .02)> to=<T t(0 0 .05)> type=JT_rigid }

body rect7 { size=[.1 .3 .1 .02] type=ST_ssBox contact }
joint (table rect7) { from=<T t(-.5 .3 .02)> to=<T t(0 0 .05)> type=JT_rigid }

body rect8 { size=[.1 .3 .1 .02] type=ST_ssBox contact }
joint (table rect8) { from=<T t(-.3 .3 .02)> to=<T t(0 0 .05)> type=JT_rigid }

#body obj3 {}
#shape (obj3){ size=[.1 .1 .2 .02] type=ST_ssBox }
#joint (table2 obj3) { type=JT_rigid from=<T t(.3 .0 .02)> to=<T t(0 0 .05) d(90 1 0 0)>}

#body obj4 { size=[.1 .1 .2 .02] type=ST_ssBox }
#joint (table2 obj4) { type=JT_rigid from=<T t(.3 .3 .02)> to=<T t(0 0 .05) d(90 1 0 0)>}

#body obj5 { size=[.1 .1 .1 .02] type=ST_ssBox }
#joint (table2 obj5) { type=JT_rigid from=<T t(.3 .6 .02)> to=<T t(0 0 .05) d(90 1 0 0)>}

shape objTarget(table2) { type=ST_marker rel=<T t(-.3 0 .02)> size=[.3 0 0 0] color=[1 .5 0] }
shape objTarget2(table2) { type=ST_marker rel=<T t(-.2 0 .02)> size=[.3 0 0 0] color=[1 .5 0] }
shape objTarget3(table2) { type=ST_marker rel=<T t(-.1 0 .02)> size=[.3 0 0 0] color=[1 .5 0] }
shape objTarget4(table2) { type=ST_marker rel=<T t(0 0 .02)> size=[.3 0 0 0] color=[1 .5 0] }
shape objTarget5(table2) { type=ST_marker rel=<T t(-.3 .1 .02)> size=[.3 0 0 0] color=[1 .5 0] }
shape objTarget6(table2) { type=ST_marker rel=<T t(-.3 .2 .02)> size=[.3 0 0 0] color=[1 .5 0] }
shape objTarget7(table2) { type=ST_marker rel=<T t(-.3 .3 .02)> size=[.3 0 0 0] color=[1 .5 0] }
#shape objTarget6(table2) { type=ST_marker rel=<T t(-.3 0 .02)> size=[.3 0 0 0] color=[1 .5 0] }
#shape objTarget7(table2) { type=ST_marker rel=<T t(-.3 .4 .02) d(90 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }

#body ball { X=<T t(-1 -1.2 1.)>, size=[.1 .1 .2 .02] type=9 }
#body ball2 { X=<T t(-1 -1.2 1.5)>, size=[.1 .1 .2 .02] type=9 }

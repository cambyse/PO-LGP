#Include = 'arm7.kvg'
Include = '../../data/pr2_model/pr2_model.ors'

body table{ type=0, X=<T t(0 -1.2 .5)>, size=[1. 1. .04 .02], color=[.8 .5 .3] }
shape leg1(table) { rel=<T t(-.25 -.25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] }
shape leg2(table) { rel=<T t(-.25  .25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] }
shape leg3(table) { rel=<T t( .25  .25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] }
shape leg4(table) { rel=<T t( .25 -.25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] }

body table2{ type=0, X=<T t(1.2 0 .5)>, size=[1. 1. .04 .02], color=[.8 .5 .3] }
shape (table2) { rel=<T t(-.25 -.25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] }
shape (table2) { rel=<T t(-.25  .25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] }
shape (table2) { rel=<T t( .25  .25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] }
shape (table2) { rel=<T t( .25 -.25 -.25)>  type=0 size=[.04 .04 .5 0] color=[0.5 0.3 0.15] }

body obj1 { size=[.1 .1 .2 .02] type=ST_ssBox }
joint (table obj1) { from=<T t(0 .3 .02)> to=<T t(0 0 .1)> type=JT_rigid }

body obj2 { size=[.1 .1 .2 .02] type=ST_ssBox X=<T t(0 -1.2 .6) t(.3 .3 .02) t(0 0 .1) t(0 0 .01)> }
joint (table obj2) { from=<T t(.3 .3 .02)> to=<T t(0 0 .1)> type=JT_rigid }

body obj3 {}
shape (obj3){ size=[.1 .1 .2 .02] type=ST_ssBox }
joint (table2 obj3) { type=JT_rigid from=<T t(.3 .0 .02)> to=<T t(0 0 .05) d(90 1 0 0)>}

body obj4 { size=[.1 .1 .2 .02] type=ST_ssBox }
joint (table2 obj4) { type=JT_rigid from=<T t(.3 .3 .02)> to=<T t(0 0 .05) d(90 1 0 0)>}

shape objTarget(table2) { type=ST_marker rel=<T t(-.3 0 .02)> size=[.3 0 0 0] color=[1 .5 0] }

body ball { X=<T t(-1 -1.2 1.)>, size=[.1 .1 .2 .02] type=9 }
body ball2 { X=<T t(-1 -1.2 1.5)>, size=[.1 .1 .2 .02] type=9 }

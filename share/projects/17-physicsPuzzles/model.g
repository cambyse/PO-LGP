body table{ type=ST_ssBox, X=<T t(0 0 .5)>, size=[1. .04 .2 .02], color=[.3 .3 .5] fixed }

body ball { type=ST_ssBox size=[.08 .08 .08 .04] color=[.8 0 0] contact }
joint (table ball)    { from=<T t(0 0.02 0)> to=<T t(0 .04 0)> type=JT_rigid }

body block { type=ST_ssBox, size=[.2 .1 .2 .02], color=[0 0 0] fixed }
joint (table block) { from=<T t(-.02 1. 0)> to=<T d(45 0 0 1) > type=JT_transY }

body box { X = <T t(.7 -.25 .5)> }

shape(box){ type=ST_ssBox rel=<T t(.15 0 0)>, size=[.04 .34 .2 .02] color=[0 .5 0] }
shape(box){ type=ST_ssBox rel=<T t(-.15 0 0)>, size=[.04 .34 .2 .02] color=[0 .5 0] }
shape(box){ type=ST_ssBox rel=<T t(0 -.15 0)>, size=[.34 .04 .2 .02] color=[0 .5 0] }


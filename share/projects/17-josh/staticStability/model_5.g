Include = '../../data/man_model.ors'




Edit waist { X=<T t(.8 0. 1.) d(-90 0 0 1)> }

body table { type=9, X=<T t(0 0 .9)>, size=[1. .8 .04 .02], color=[.3 .3 .5] fixed }

shape humanR (handR){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }
shape humanL (handL){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }

body yellow { type=9 size=[.05 .05 .1 .005] color=[.8 .8 0] contact }
body red    { type=9 size=[.05 .05 .1 .005] color=[.8 0 0] contact }
body blue   { type=9 size=[.05 .05 .10 .005] color=[0 0 .8] contact }



joint (table yellow) { from=<T t(0   0 0.02) t(0 0 .05)> type=JT_rigid }
joint (blue red)    { from=<T d(-45 1 0 0) t(0 .025 .08)> type=JT_rigid }
joint (table blue)   { from=<T t(0 -.04 .04)  d(45 1 0 0) t(0 0 .05) > type=JT_rigid }
#joint (table yellow) { from=<T t(0 -0.078 0.02)> to=<T t(0 0 .05) d(-45 45 0 1)> type=JT_rigid }

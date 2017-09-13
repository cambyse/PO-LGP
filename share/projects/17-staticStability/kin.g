Include = '../../data/baxter_model/baxter.ors'

shape baxterR (right_wrist){ rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> type=5 size=[.1 0 0 0] color=[1 1 0] }
shape baxterL (left_wrist) { rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> type=5 size=[.1 0 0 0] color=[1 1 0] }

body table{ type=9, X=<T t(1. 0 .7)>, size=[1.5 2. .04 .02], color=[.3 .3 .3] fixed, contact }

body yellow { type=9 size=[.05 .2 .05 .005] color=[.8 .8 0] contact }
body red    { type=9 size=[.05 .05 .1 .005] color=[.8 0 0] contact }
body blue   { type=9 size=[.02 .02 .1 .005] color=[0 0 .8] contact }
body green  { type=9 size=[.05 .05 .1 .005] color=[0 .8 0] contact, mass=3. }

joint (table yellow) { from=<T t(-.3 -.1 0.02) t(0 0 .025)> type=JT_rigid }
joint (table red)    { from=<T t(-.3 .2 0.02) t(0 0 .05)> type=JT_rigid }
joint (table blue)   { from=<T t(-.3 .1 .02) t(0 0 .05)> type=JT_rigid }
joint (table green)  { from=<T t(-.3 .3 .02) t(0 0 .05)> type=JT_rigid }

shape plate(yellow) { rel=<T t(0 0 .025) t(0 .075 .005)> type=9 size=[.05 .05 .01 .005] color=[0 1 1] contact }

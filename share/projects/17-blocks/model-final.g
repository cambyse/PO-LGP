Include = '../../data/man_model.ors'

Edit waist { X=<T t(.8 0. 1.) d(-90 0 0 1)> }

body tableC{ type=9, X=<T t(0 0 .9)>, size=[1. .8 .04 .02], color=[.3 .3 .5] fixed, contact }

shape humanR (handR){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }
shape humanL (handL){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }

body red { type=9 size=[.05 .1 .05 .005] color=[.8 0 0] contact }
joint (tableC red) { from=<T t(0 0 0.02)> to=<T t(0 0 .025)> type=10 }

body blue { type=9 size=[.05 .05 .10 .005] color=[0 0 .8] contact }
joint (red blue) { from=<T t(0 0 .025)> to=<T t(0 0 .05)> type=JT_transXYPhi }

body yellow { type=9 size=[.05 .1 .05 .005] color=[.8 .8 0] contact }
joint (blue yellow) { from=<T t(0 0 0.05)> to=<T t(0 0 .025)> type=JT_transXYPhi }


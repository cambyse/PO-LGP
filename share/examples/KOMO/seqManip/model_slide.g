#Include = 'arm7.kvg'
#Include = '../../../data/pr2_model/pr2_model.ors'

body table{ type=0, X=<T t(0 -1.2 .6)>, size=[1. 1. .04 .0], color=[.8 .5 .3] fixed,  }
shape leg1(table) { rel=<T t(-.3 -.3 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg2(table) { rel=<T t(-.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg3(table) { rel=<T t(.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg4(table) { rel=<T t(.3 -.3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }

body slider1a { type=ST_box size=[.2 .1 .05 0] color=[0 0 0] }
body slider1b { type=ST_box size=[.2 .1 .05 0] color=[1 0 0] }
joint (slider1a slider1b){ type=JT_transX }
shape slider1Eff (slider1b){ rel=<T t(.1 0 0)> type=5 size=[.1 .1 .1] color=[0 1 0] }

body obj1 { size=[.1 .1 .2 .02] type=9 contact }
joint (table obj1) { from=<T t(0 0 .02)> to=<T t(-.2 .4 .1)> type=JT_rigid }

shape target(table){ type=5 rel=<T t(0 .3 .15)> size=[.3 0 0 0] color=[1 1 0]}

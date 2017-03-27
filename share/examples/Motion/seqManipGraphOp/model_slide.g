#Include = 'arm7.kvg'
#Include = '../../../data/pr2_model/pr2_model.ors'

body table{ type=0, X=<T t(0 -1.2 .6)>, size=[1. 1. .04 .0], color=[.8 .5 .3] fixed,  }
shape leg1(table) { rel=<T t(-.3 -.3 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg2(table) { rel=<T t(-.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg3(table) { rel=<T t(.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg4(table) { rel=<T t(.3 -.3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }

#body slider1 { type=ST_box size=[.2 .1 .05 0] color=[0 0 0] }
#body slider2 { type=ST_box size=[.2 .1 .05 0] color=[1 0 0] }
#joint (table slider1){ type=JT_rigid }
#joint (table slider1){ type=JT_transXYPhi }
#joint (slider1 slider2){ type=JT_transX }

body obj1 { size=[.1 .1 .2 .02] type=9 contact }

#joint (slider1 obj1){ type=JT_hingeZ from=<T t(0 0 .02)> to=<T t(0 0 .1)> }
joint (table obj1) { from=<T t(0 0 .02)> to=<T t(-.2 .4 .1)> type=JT_rigid }

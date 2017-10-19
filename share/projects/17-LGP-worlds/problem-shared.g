#Include = '../../data/baxter_model/baxter.ors'
Include='../../../share/data/drake_kuka/setup.g'
Edit base { X=<T t(.4 0 .7)> }

body slider1a { type=ST_box size=[.2 .02 .02 0] color=[.5 .5 .5] }
body slider1b { type=ST_box size=[.2 .02 .02 0] color=[.8 .3 .3] }
joint slider1Joint(slider1a slider1b){ type=JT_transX }
shape (slider1b){ rel=<T t(.1 0 0)> type=5 size=[.1 .1 .1] color=[0 1 0] }

body table1{ type=9, X=<T t(1.2 0 .7)>, size=[2. 3. .04 .02], color=[.3 .3 .3] fixed, contact }

#shape baxterR (right_wrist){ rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> type=5 size=[.1 0 0 0] color=[1 1 0] }
#shape baxterL (left_wrist) { rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> type=5 size=[.1 0 0 0] color=[1 1 0] }


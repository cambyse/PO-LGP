Include = '../../../data/pr2_model/pr2_model.ors'

body wall1{ type=9, X=<T t(0 .75 .5)>, size=[1. .04 1. .02], color=[.8 .5 .3] contact }
body wall2{ type=9, X=<T t(0 -.75 .5)>, size=[1. .04 1. .02], color=[.8 .5 .3] contact }

body table{ type=0, X=<T t(.7 0 .5)>, size=[1. 1. .04 .02], color=[.8 .5 .3] contact }


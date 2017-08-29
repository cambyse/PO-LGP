Include = '../../data/pr2_model/pr2_model.ors'

body table1{ type=9, X=<T t(1. 0 .5)>, size=[1.2 2. .04 .02], color=[.3 .3 .3] }

body table_coll{ type=9, X=<T t(1. 0 .5)>, size=[1.4 2.2 .24 .1], color=[1 0 0 .3] contact }

body targetBody{ type=ST_marker, size=[.5], color=[0 1 0] }
joint target(world targetBody){ type=JT_transXYPhi, Q=<T t(2. 0 0) d(180 0 0 1)> }

Uncertainty (worldTranslationRotation){ sigma=[.1 .1 .5] }

body landmark{ type=ST_marker, size=[.5], X=<T t(1.2 2. 2.)> }

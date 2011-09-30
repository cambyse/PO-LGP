
# convert data to grid data (for splot)
# dgrid3d <Xresolution> <Yresolution> <surface regularizer>
#<resolution>:
#	higher resolution costs more time.
# <regularizer>:
#	weight of point dependent on distance to surface;
# 	lower = smoother surface;
# 	higer = better approximation.
# 	Hiher values cost time. 
set dgrid3d 20,20, 8
#set dgrid3d 40,40, 16

# where to draw the contour and for which thresholds
set cntrparam levels discrete  0
set contour 
set lines

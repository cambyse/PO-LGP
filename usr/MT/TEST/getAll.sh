where="../Core ../Kin ../Geo ../GeoOptim ../Gui ../Optim ../Plot ../Algo ../KOMO"
echo "#include <GL/glew.h>" > all.cpp
cat `find $where -maxdepth 1 -name '*.cpp' -printf "%p "` >> all.cpp

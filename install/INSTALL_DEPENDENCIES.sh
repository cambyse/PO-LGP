# install ubuntu packages
sudo apt-get install \
bison \
build-essential \
doxygen \
fabric \
flex \
freeglut3-dev \
g++ \
gcc \
gfortran \
git-core \
gnuplot \
graphviz-dev \
libann-dev \
libcv-dev \
libcvaux-dev \
libdc1394-22-dev \
libf2c2-dev \
libgl1-mesa-dev \
libgl1-mesa-glx \
libglu1-mesa \
libglu1-mesa-dev \
libgtest-dev\
libgtkglext1-dev \
libhighgui-dev \
liblapack-dev \
libplib-dev \
libqhull-dev \
libsdl1.2-dev \
libx11-dev \
libx11-dev \
libxi-dev \
libxmu-dev \
make \
meld \
python-nose \
python-unittest2 \
realpath \
regexxer \
swig2.0 \
tcl8.5-dev \
tk-dev \
tk8.5-dev \


# install google test
cd /usr/src/gtest
sudo cmake .
sudo make
sudo ln -s -f /usr/src/gtest/libgtest.a /home/lib/lib/
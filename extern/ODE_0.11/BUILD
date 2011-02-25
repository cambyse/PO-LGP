wget http://sourceforge.net/projects/opende/files/ODE/0.11/ode-0.11.tar.gz/download
tar xvzf download

cd ode-0.11
sh autogen.sh
#./configure --help
./configure --prefix=$PWD --enable-double-precision --with-trimesh=opcode --with-pic
make clean -j
make -j

cd ..
rm -Rf lib include
mkdir lib include include/internal include/internal/joints
cp ode-0.11/ode/src/.libs/libode.a ode-0.11/ode/src/.libs/libfast.a lib
cp -R ode-0.11/include/ode include
cp -R ode-0.11/ode/src/*.h include/internal
cp -R ode-0.11/ode/src/joints/*.h include/internal/joints

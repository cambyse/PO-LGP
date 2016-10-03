cd src

find . \( -name '*.h' -or -name '*.cpp' -or -name '*.tpp' \) -exec ../bin/redoSrcCopyrightHeaders.filter {} \;

headache -h ../bin/redoSrcCopyrightHeaders.header -c ../bin/redoSrcCopyrightHeaders.conf \
Core/*.h Core/*.cpp Core/*.tpp \
Algo/*.h Algo/*.cpp \
Gui/*.h Gui/*.cpp \
Ors/*.h Ors/*.cpp \
Optim/*.h Optim/*.cpp \
Motion/*.h Motion/*.cpp \
FOL/*.h FOL/*.cpp \
MCTS/*.h MCTS/*.cpp \
Geo/*.h Geo/*.cpp \
LGP/*.h LGP/*.cpp \

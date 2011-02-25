PACKAGE = libORS

all: lib/extern/SWIFT++_1.2/src lib/extern/ANN_1.1/src lib/src/MT/ lib/src/NP lib/src/Lewiner 

cleanAll: clean/extern/SWIFT++_1.2/src clean/extern/ANN_1.1/src clean/src/MT/ clean/src/NP clean/src/Lewiner cleanProjects

tests::
	-@find test -maxdepth 1 -type d -not -name 'test' -exec make -C {} \;
	-@find . -name 'roboticsCourse*' -maxdepth 1 -type d -exec make -C {} \;

lib/%::
	make -C $*


clean/%::
	make clean -C $*

cleanProjects::
	-@find test -maxdepth 1 -type d -not -name 'test' -exec make -C {} clean \;
	-@find .    -maxdepth 1 -type d -name 'roboticsCourse*' -exec make -C {} clean \;

zip::
	cd ..;  rm -f $(PACKAGE).tgz;  tar cvzf $(PACKAGE).tgz $(PACKAGE) --exclude "*.svn" --exclude "*solution*" --exclude "*ODE_0.11/ode-0.11" --exclude "*ODE_0.11/include" --exclude "*ODE_0.11/lib" --exclude "*/IBDS*"

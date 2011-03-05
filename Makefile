PACKAGE = libORS

ALL =	extern/SWIFT++_1.2/src \
	extern/ANN_1.1/src \
	extern/libcolorseg/src \
	src/MT \
	src/NP \
	src/Lewiner 

makeAll: $(ALL:%=make/%)

cleanAll: $(ALL:%=clean/%)

make/%::
	make -C $*

clean/%::
	make clean -C $*

tests::
	-@find test -maxdepth 1 -type d -not -name 'test' -exec make -C {} \;
	-@find . -name 'roboticsCourse*' -maxdepth 1 -type d -exec make -C {} \;

cleanProjects::
	-@find test -maxdepth 1 -type d -not -name 'test' -exec make -C {} clean \;
	-@find .    -maxdepth 1 -type d -name 'roboticsCourse*' -exec make -C {} clean \;

zip::
	cd ..;  rm -f $(PACKAGE).tgz;  tar cvzf $(PACKAGE).tgz $(PACKAGE) --exclude "*.svn" --exclude "*solution*" --exclude "*ODE_0.11/ode-0.11" --exclude "*ODE_0.11/include" --exclude "*ODE_0.11/lib" --exclude "*/IBDS*"

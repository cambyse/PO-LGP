PACKAGE = share

ALL =	src/NP \
	src/MT

makeAll: $(ALL:%=make/%)

cleanAll: $(ALL:%=clean/%) cleanProjects

make/%::
	make -C $*

clean/%::
	make clean -C $*

tests::
	-@find test -maxdepth 1 -type d -not -name 'test' -exec make -C {} \;

cleanProjects::
	-@find test -maxdepth 1 -type d -not -name 'test' -exec make -C {} clean \;
	-@find robot -maxdepth 1 -type d -not -name 'robot' -exec make -C {} clean \;

doc::
	make -C doc guide doxy

zip::
	cd ..;  rm -f $(PACKAGE).tgz;  tar cvzf $(PACKAGE).tgz $(PACKAGE) --exclude "*.svn" --exclude ".git" --exclude "*solution*" --exclude "*ODE_0.11/ode-0.11" --exclude "*ODE_0.11/include" --exclude "*ODE_0.11/lib" --exclude "*/IBDS*" --exclude "*ors_mesh*"

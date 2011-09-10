PACKAGE = share

ALL =	extern/SWIFT++_1.2/src \
	extern/libcolorseg/src \
	extern/Lewiner \
	src/NJ \
	src/TL \
	src/NP \
	src/SD \
	src/DZ \
	src/MT \
#	extern/SWIFT++_1.2/decomposer/src \

makeAll: $(ALL:%=make/%)

cleanAll: $(ALL:%=clean/%) cleanProjects

reformatSources:
	astyle --options=src/style.astyle "src/MT/*.h" "src/MT/*.cpp" "src/MT/*.cxx"
	cd src; find MT/ \( -name "*.h" -or -name "*.cpp" -or -name "*.cxx" \) -exec ./style.sed.sh {} \;

make/%::
	make -C $*

clean/%::
	make clean -C $*

tests::
	-@find test -maxdepth 1 -type d -not -name 'test' -exec make -C {} \;

projects::
	-@find proj_* -maxdepth 1 -type d -not -name 'robot' -exec make -C {} \;

cleanProjects::
	-@find test -maxdepth 1 -type d -not -name 'test' -exec make -C {} clean \;
	-@find proj_* -maxdepth 1 -type d -not -name 'robot' -exec make -C {} clean \;

doc::
	make -C doc guide doxy

zip::
	cd ..;  rm -f $(PACKAGE).tgz;  tar cvzf $(PACKAGE).tgz $(PACKAGE) --exclude "*.svn" --exclude ".git" --exclude "*solution*" --exclude "*ODE_0.11/ode-0.11" --exclude "*ODE_0.11/include" --exclude "*ODE_0.11/lib" --exclude "*/IBDS*" --exclude "*ors_mesh*"

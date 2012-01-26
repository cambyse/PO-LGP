NAME   = $(shell basename $(PWD))

MLR_LIBS = \
	src/MT \
	src/SD \
	src/NJ \
	src/relational \
#	src/DZ \

EXTERN = \
	extern/libcolorseg/src \
	extern/Lewiner \
	extern/SWIFT++_1.2/src \
#	extern/SWIFT++_1.2/decomposer/src \


makeAll: makeEXTERN $(MLR_LIBS:%=make/%)

cleanAll: $(MLR_LIBS:%=clean/%) cleanProjects

makeEXTERN: $(EXTERN:%=make/%)


reformatSources:
	astyle --options=src/style.astyle "src/MT/*.h" "src/MT/*.cpp" "src/MT/*.cxx"
	cd src; find MT/ \( -name "*.h" -or -name "*.cpp" -or -name "*.cxx" \) -exec ./style.sed.sh {} \;

make/%::
	make -C $*

clean/%::
	make clean -C $*

tests::
	-@find test -maxdepth 1 -follow -type d -not -name 'test' -not -name 'SHARE' -exec make -C {} \;

projects::
	-@find proj_* -maxdepth 1 -follow -type d -not -name 'proj_*' -not -name 'SHARE' -exec make -C {} \;

cleanProjects::
	-@find test -maxdepth 1 -follow -type d -not -name 'test' -not -name 'SHARE' -exec make -C {} clean \;
	-@find proj_* -maxdepth 1 -follow -type d -not -name 'robot' -not -name 'SHARE' -exec make -C {} clean \;

doc::
	make -C doc guide doxy

zip::
	cd ..;  rm -f $(NAME).tgz;  tar cvzf $(NAME).tgz $(NAME) --dereference --exclude "SHARE" --exclude ".git" --exclude "*.kdev4" --exclude "xx-*" --exclude "MT.log" --exclude "*solution*" --exclude "course/*/main.cpp"  --exclude "pics" --exclude "*ODE_0.11/ode-0.11" --exclude "*ODE_0.11/include" --exclude "*ODE_0.11/lib" --exclude "*/IBDS*" --exclude "*ors_mesh*"

NAME   = $(shell basename $(PWD))

ALL =	extern/SWIFT++_1.2/src \
	src/MT
#	extern/SWIFT++_1.2/decomposer/src \

makeAll: $(ALL:%=make/%) tests courses

cleanAll: $(ALL:%=clean/%) cleanProjects

make/%::
	make -C $*

clean/%::
	make clean -C $*

tests::
	-@find test -maxdepth 1 -follow -type d -not -name 'test' -not -name 'SHARE' -exec make -C {} \;

courses::
	-@find course -maxdepth 1 -follow -type d -not -name 'course' -not -name 'SHARE' -exec make -C {} \;

cleanProjects::
	-@find test -maxdepth 1 -follow -type d -not -name 'test' -not -name 'SHARE' -exec make -C {} clean \;
	-@find course -maxdepth 1 -follow -type d -not -name 'course' -not -name 'SHARE' -exec make -C {} clean \;

doc::
	make -C doc guide doxy

zip::
	cd ..;  rm -f $(NAME).tgz;  tar cvzf $(NAME).tgz $(NAME) --dereference --exclude "SHARE" --exclude ".git" --exclude "*.kdev4" --exclude "MT.log" --exclude "*solution*" --exclude "course/*/main.cpp" --exclude "*ODE_0.11/ode-0.11" --exclude "*ODE_0.11/include" --exclude "*ODE_0.11/lib" --exclude "*/IBDS*" --exclude "*ors_mesh*"

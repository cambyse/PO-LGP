BASE = $(shell pwd)/..

test::
	@find . -maxdepth 1 -mindepth 1 -type d -not -name '.svn' -exec make {}/x.exe \;

%/x.exe::
	@echo; echo; echo ----------------------------------------
	@echo "     " $*;
	@echo ----------------------------------------
	@echo; echo ----- compiling
	-make -w -C $*
	@echo; echo ----- testing
	-cd $*; ./x.exe;

clean::
	@find . -maxdepth 1 -mindepth 1 -type d -not -name '.svn' -exec make -w -C {} clean \;
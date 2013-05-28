NAME   = $(shell basename $(PWD))

srcs = $(shell find src -mindepth 1 -maxdepth 1 -type d)
exas = $(shell find examples -mindepth 2 -maxdepth 2 -type d)

################################################################################

all: $(srcs:%=make/%) $(exas:%=make/%)

depend: $(srcs:%=depend/%)

clean: $(srcs:%=clean/%) $(srcs:%=clean/%) 

################################################################################

make/%::
	@echo
	@echo "                                                  ***** make " $*
	$(MAKE) -C $* --no-print-directory

depend/%::
	$(MAKE) -C $* depend --no-print-directory

clean/%::
	-rm -f $*/Makefile.dep
	$(MAKE) -C $* clean --no-print-directory 

################################################################################

doc::
	#$(MAKE) -w -C doc guide doxy
	cd doc; doxygen MLR.doxy;

reformatSources:
	astyle --options=src/style.astyle "src/MT/*.h" "src/MT/*.cpp" "src/MT/*.cxx"
	cd src; find MT/ \( -name "*.h" -or -name "*.cpp" -or -name "*.cxx" \) -exec ./style.sed.sh {} \;


zip::
	cd ..;  rm -f $(NAME).tgz;  tar cvzf $(NAME).tgz $(NAME) --dereference --exclude "SHARE" --exclude ".git" --exclude "*.kdev4" --exclude "xx-*" --exclude "MT.log" --exclude "*solution*" --exclude "course/*/main.cpp"  --exclude "pics" --exclude "*ODE_0.11/ode-0.11" --exclude "*ODE_0.11/include" --exclude "*ODE_0.11/lib" --exclude "*/IBDS*" --exclude "*ors_mesh*"

%.tgz:: force
	rm -f $*.tgz;
	cd ..; \
	tar cvzf $*.tgz --exclude-vcs \
	--files-from $(PWD)/slices/tar.incl \
	--files-from $(PWD)/slices/$*.tar.incl \
	--exclude-from $(PWD)/slices/tar.excl \
	--exclude-from $(PWD)/slices/$*.tar.excl

force: ;
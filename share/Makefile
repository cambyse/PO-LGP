NAME   = $(shell basename $(PWD))

# The order of the libs matters!
MLR_LIBS = \
	src/MT \
	src/JK \
	src/views \
	src/biros \
	src/motion \
	src/hardware \
	src/perception \
	src/relational \

EXTERN = \
	extern/libcolorseg/src \
	extern/Lewiner \
	extern/ply \
	extern/tony_mdp \
	extern/SWIFT++_1.2/src \
	extern/SWIFT++_1.2/decomposer/src \


all: extern $(MLR_LIBS:%=make/%)

depend: $(MLR_LIBS:%=depend/%)

extern: $(EXTERN:%=make/%)

cleanAll: $(MLR_LIBS:%=clean/%) cleanProjects
	find . -maxdepth 3 -name 'Makefile.dep' -delete

cleanDepend:
	find . -maxdepth 3 -name 'Makefile.dep' -delete

cleanExtern: $(EXTERN:%=clean/%)


reformatSources:
	astyle --options=src/style.astyle "src/MT/*.h" "src/MT/*.cpp" "src/MT/*.cxx"
	cd src; find MT/ \( -name "*.h" -or -name "*.cpp" -or -name "*.cxx" \) -exec ./style.sed.sh {} \;

make/%::
	$(MAKE) -C $*

depend/%::
	$(MAKE) -C $* depend

clean/%::
	-rm -f $*/Makefile.dep
	$(MAKE) clean -C $*

tests::
	-@find test -maxdepth 1 -follow -type d -not -name 'test' -not -name 'SHARE' -exec $(MAKE) -C {} \;

demo::
	-@find demo -maxdepth 1 -follow -type d -not -name 'demo' -not -name 'SHARE' -exec $(MAKE) -C {} \;

cleanProjects::
	-@find test -maxdepth 1 -follow -type d -not -name 'test' -not -name 'SHARE' -exec $(MAKE) -C {} clean \;
	-@find demo -maxdepth 1 -follow -type d -not -name 'demo' -not -name 'SHARE' -exec $(MAKE) -C {} clean \;

doc::
	$(MAKE) -C doc guide doxy

zip::
	cd ..;  rm -f $(NAME).tgz;  tar cvzf $(NAME).tgz $(NAME) --dereference --exclude "SHARE" --exclude ".git" --exclude "*.kdev4" --exclude "xx-*" --exclude "MT.log" --exclude "*solution*" --exclude "course/*/main.cpp"  --exclude "pics" --exclude "*ODE_0.11/ode-0.11" --exclude "*ODE_0.11/include" --exclude "*ODE_0.11/lib" --exclude "*/IBDS*" --exclude "*ors_mesh*"

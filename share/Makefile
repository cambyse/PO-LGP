BASE = .

target: examples

################################################################################

NAME   = $(shell basename $(PWD))

srcs =  $(shell find src -mindepth 1 -maxdepth 1 -type d -not -name 'extern' )
srcs += $(shell find src/extern -mindepth 1 -maxdepth 1 -type d)
exas = $(shell find examples -mindepth 2 -maxdepth 2 -type d -not -name '*py')
pysrcs = src/Core src/Ors src/Gui src/Motion src/Optim

################################################################################

all: $(srcs:%=makePath/%) $(exas:%=makePath/%)

src: $(srcs:%=makePath/%)

python: $(srcs:%=makePath/%) $(pysrcs:%=makePythonPath/%)

examples: $(exas:%=makePath/%)

depend: $(srcs:%=dependPath/%)

clean: $(srcs:%=cleanPath/%) $(exas:%=cleanPath/%) cleanLocks

inf:
	@echo "srcs =" $(srcs)

################################################################################

doc::
	cd doc; doxygen MLR.doxy;
#	$(MAKE) -w -C doc guide doxy

reformatSources:
	astyle --options=src/style.astyle "src/MT/*.h" "src/MT/*.cpp" "src/MT/*.cxx"
	cd src; find MT/ \( -name "*.h" -or -name "*.cpp" -or -name "*.cxx" \) -exec ./style.sed.sh {} \;

cleanLocks: force
	find . -name 'Make.lock' -type d -delete

%.tgz:: force
	mv -f make-config make-config.mine
	cp -f slices/$*.make-config ./make-config
	rm -f $*.tgz;
	cd ..; \
	tar cvzf $*.tgz --exclude-vcs \
	--files-from $(PWD)/slices/tar.incl \
	--files-from $(PWD)/slices/$*.tar.incl \
	--exclude-from $(PWD)/slices/tar.excl \
	--exclude-from $(PWD)/slices/$*.tar.excl
	cp -f make-config.mine make-config

include make-generic

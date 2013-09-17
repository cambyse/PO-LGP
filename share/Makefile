BASE = .

target: src

################################################################################

NAME   = $(shell basename $(PWD))

srcs =  $(shell find src -mindepth 1 -maxdepth 1 -type d -not -name 'extern')
srcs += $(shell find src/extern -mindepth 1 -maxdepth 1 -type d)
#srcs += $(shell find src/Hardware -mindepth 1 -maxdepth 1 -type d)
exas = $(shell find examples -mindepth 2 -maxdepth 2 -type d)

################################################################################

all: $(srcs:%=makePath/%) $(exas:%=makePath/%)

src: $(srcs:%=makePath/%)

examples: $(exas:%=makePath/%)

depend: $(srcs:%=dependPath/%)

clean: $(srcs:%=cleanPath/%) $(exas:%=cleanPath/%) cleanLocks

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
	rm -f $*.tgz;
	cd ..; \
	tar cvzf $*.tgz --exclude-vcs \
	--files-from $(PWD)/slices/tar.incl \
	--files-from $(PWD)/slices/$*.tar.incl \
	--exclude-from $(PWD)/slices/tar.excl \
	--exclude-from $(PWD)/slices/$*.tar.excl

include make-generic

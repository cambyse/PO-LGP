NAME   = $(shell basename $(PWD))

ALL =	src/MT \
        src/biros \
        use_cases/vision/extern/libcolorseg/src

makeAll: $(ALL:%=make/%) makePaths/useCases

cleanAll: $(ALL:%=clean/%) cleanPaths/UseCases

make/%::
	make -C $*

clean/%::
	make clean -C $*

makePaths/%::
	-@find -L $* -maxdepth 1 -follow -type d -not -name '$*' -not -name 'SHARE' -exec make -C {} \;

cleanPaths/%::
	-@find -L $* -maxdepth 1 -follow -type d -not -name '$*' -not -name 'SHARE' -exec make -C {} clean \;

doc::
	make -C doc doxy_array

zip::
	cd ..;  rm -f $(NAME).tgz;  tar cvzf $(NAME).tgz $(NAME) --dereference --exclude-vcs --exclude-from tar.exclude

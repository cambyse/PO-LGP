NAME   = $(shell basename $(PWD))

ALL =	extern/SWIFT++_1.2/src \
	src/MT
#	extern/SWIFT++_1.2/decomposer/src \

makeAll: $(ALL:%=make/%) makePaths/test makePaths/course

cleanAll: $(ALL:%=clean/%) cleanPaths/test cleanPaths/course

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

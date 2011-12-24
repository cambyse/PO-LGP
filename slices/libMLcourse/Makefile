NAME   = $(shell basename $(PWD))

ALL =	src/MT

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
	make -C doc doxy_array

zip::
	cd ..;  rm -f $(NAME).tgz;  tar cvzf $(NAME).tgz $(NAME) --dereference --exclude-vcs --exclude-from tar.exclude

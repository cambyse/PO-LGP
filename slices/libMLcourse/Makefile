NAME   = $(shell basename $(PWD))

ALL =	src/MT

makeAll: $(ALL:%=make/%) tests courses doc

cleanAll: $(ALL:%=clean/%) cleanProjects

make/%::
	make -C $*

clean/%::
	make clean -C $*

tests::
	-@find -L test -maxdepth 1 -follow -type d -not -name 'test' -not -name 'SHARE' -exec make -C {} \;

courses::
	-@find -L course -maxdepth 1 -follow -type d -not -name 'course' -not -name 'SHARE' -exec make -C {} \;

cleanProjects::
	-@find -L test -maxdepth 1 -follow -type d -not -name 'test' -not -name 'SHARE' -exec make -C {} clean \;
	-@find -L course -maxdepth 1 -follow -type d -not -name 'course' -not -name 'SHARE' -exec make -C {} clean \;

doc::
	make -C doc doxy_array

zip::
	cd ..;  rm -f $(NAME).tgz;  tar cvzf $(NAME).tgz $(NAME) --dereference --exclude-vcs --exclude-from tar.exclude --exclude-from $(NAME)/tar.exclude

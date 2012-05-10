NAME   = $(shell basename $(PWD))

ALL = src/relational src/MT

makeAll: $(ALL:%=make/%) tests doc

cleanAll: $(ALL:%=clean/%) cleanTests clean/doc clean/lib

make/%::
	make -C $*

clean/%::
	make clean -C $*

tests::
	-@find -L test -maxdepth 1 -follow -type d -not -name 'test' -not -name 'SHARE' -exec make -C {} \;

cleanTests::
	-@find -L test -maxdepth 1 -follow -type d -not -name 'test' -not -name 'SHARE' -exec make -C {} clean \;

doc::
	make -C doc doxy_array

zip::
	cd ..;  rm -f $(NAME).tgz;  tar cvzf $(NAME).tgz $(NAME) --dereference --exclude-vcs --exclude-from tar.exclude --exclude-from $(NAME)/tar.exclude

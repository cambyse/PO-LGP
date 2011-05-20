NAME   = $(shell basename $(PWD))

ALL =	src/MT

makeAll: $(ALL:%=make/%)

cleanAll: $(ALL:%=clean/%) cleanProjects

make/%::
	make -C $*

clean/%::
	make clean -C $*

tests::
	-@find test -maxdepth 1 -type d -not -name 'test' -exec make -C {} \;

cleanProjects::
	-@find test -maxdepth 1 -type d -not -name 'test' -exec make -C {} clean \;
	-@find robot -maxdepth 1 -type d -not -name 'robot' -exec make -C {} clean \;

doc::
	make -C doc guide doxy

zip::
	cd ..;  rm -f $(NAME).tgz;  tar cvzf $(NAME).tgz $(NAME) --dereference --exclude "SHARE" --exclude "*.svn" --exclude ".git" --exclude "cudaSDK/*" --exclude "opencv-2.1/*" --exclude "cuda/*"

BASE = $(shell pwd)/..

compile:
	@for a in $$(ls); do \
		if [ -d $$a ]; then \
			echo ----------------------------------------; \
			echo "        $$a"; \
			echo ----------------------------------------; \
			$(MAKE) -C $$a; \
			echo; echo; \
		fi; \
	done;

test:
	@for a in $$(ls); do \
		if [ -d $$a ]; then \
			echo ----------------------------------------; \
			echo "        $$a"; \
			echo ----------------------------------------; \
			echo " compile ------------------------------"; \
			$(MAKE) -C $$a; \
			echo " test ---------------------------------"; \
			$$a/x.exe; \
			echo; echo; \
		fi; \
	done;

clean::
	@find . -maxdepth 1 -mindepth 1 -type d -not -name '.svn' -exec make -w -C {} clean \;

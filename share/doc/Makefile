LH = latex2html -split 0 -no_navigation
# see also /usr/local/lib/latex2html/l2hconf.pm

PWD := $(shell pwd)

default: doxy_ors doxy_array doxy_soc

doxy_%::
	doxygen doxy/$*.doxy
# 1> /dev/null 2> /dev/null

guide::
	-latex -interaction=nonstopmode '\newcommand{\pwd}{$(PWD)}\input{guide.tex}' > /dev/null
	-pdflatex -interaction=nonstopmode '\newcommand{\pwd}{$(PWD)}\input{guide.tex}' > /dev/null
# 	@echo
# 	@echo "** warnings **"
# 	@echo
# 	-@grep "Warning" guide.log
# 	-@grep -A2 "Undefined control sequence" guide.log
# 	-@grep -A3 "Missing" guide.log
# 	-@grep "Error" guide.log
#	$(LH) guide.tex
#	-latex -interaction=nonstopmode guide.tex

source::
	cd ../src/MT; \
		ctags --excmd=n --extra=+f+q --file-scope=no \
		--language-force=C++ --recurse=yes --C++-kinds=-n-l-v-d-e-m \
		*.h *.cpp
	cd ../src/MT; \
		source-highlight --src-lang cpp --out-format html \
		--line-number=' ' --line-number-ref   \
		--doc --title=libMT --tab=2 --ctags="" \
		--style-file=source.style \
		*.h *.cpp
	mkdir source
	mv -f ../src/MT/*.html source
#		--gen-references=inline --ctags="" \
#		--style-css-file=sh_print.css \

html::
	cd ../src; htags -g -F
	rm -Rf ./HTML
	mv ../src/HTML .
	rm ../src/GPATH ../src/GRTAGS ../src/GSYMS ../src/GTAGS


clean::
	rm -Rf guide html HTML *.log *.aux *.out *.toc *.blg guide.dvi

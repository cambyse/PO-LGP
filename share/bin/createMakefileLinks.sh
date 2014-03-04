echo "creating makefile -> Makefile.gof everywhere"
find . -name 'Makefile.gof' -execdir ln -f -s {} makefile \;
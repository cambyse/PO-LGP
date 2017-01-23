echo "creating makefile -> Makefile everywhere"
find .. -name 'Makefile' -execdir ln -f -s {} makefile \;

GOF=${1%/*}/Makefile.gof
LINK=${1%/*}/makefile
echo linking $GOF to $LINK
cd ${1%/*}; rm makefile; ln -s Makefile.gof makefile
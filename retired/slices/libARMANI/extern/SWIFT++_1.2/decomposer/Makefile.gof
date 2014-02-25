decomposer_c: RAPID/libRAPIDdecomp.a
	cd src; $(MAKE) CC="$(CC)" CFLAGS="$(CFLAGS) -DSWIFT_ALLOW_BOUNDARY" QHULL_DIR="$(QHULL_DIR)" decomposer_c; cd ..
	mv src/decomposer bin
	rm -f src/*.o RAPID/*.a ../lib/libSWIFT++decomp.a

decomposer_g: RAPID/libRAPIDdecomp.a
	cd src; $(MAKE) CC="$(CC)" CFLAGS="$(CFLAGS) -DDECOMP_GRAPHICS -DSWIFT_ALLOW_BOUNDARY" QHULL_DIR="$(QHULL_DIR)" decomposer_g; cd ..
	mv src/decomposer bin
	rm -f src/*.o RAPID/*.a ../lib/libSWIFT++decomp.a

RAPID/libRAPIDdecomp.a:
	cd RAPID; $(MAKE) CC="$(CC)" CFLAGS="$(CFLAGS)"; cd ..
	rm -f RAPID/*.o

depend:
	cd src; $(MAKE) QHULL_DIR="$(QHULL_DIR)" depend; cd ..

clean:
	rm -f */*.o */*.a


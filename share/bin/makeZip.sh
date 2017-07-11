#/bin/sh

#arguments: slice name

DATE=`date +%y-%m-%d`

# save the config
if [ ! -f build/config.mk.$DATE.tmp ]; then
    mv -f build/config.mk build/config.mk.$DATE.tmp
fi

# copy some files into place
cp -f slices/$1.config.mk ./build/config.mk
cp -f slices/$1.README.md README.md
cp -f ../README.md README.mlr.md
cp -f ../COPYING .
cp -f ../install/INSTALL_ALL_UBUNTU_PACKAGES.sh .
rm -f $1.tgz;

# go one path down and zip, renaming share to $1

cd ..
tar cvzfh $1.tgz --transform "s/^share/$1/" --exclude-vcs \
    --files-from share/slices/default.incl \
    --files-from share/slices/$1.incl \
    --exclude-from share/slices/default.excl \
    --exclude-from share/slices/$1.excl

# undo
cd share
cp -f build/config.mk.$DATE.tmp build/config.mk
rm -f README.md README.mlr.md COPYING INSTALL_ALL_UBUNTU_PACKAGES.sh

# unzip in z.$1 for inspection
rm -Rf z.$1; mkdir z.$1; cd z.$1; tar xzf ~/git/mlr/$1.tgz

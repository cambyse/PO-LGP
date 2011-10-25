#cp $1 $1.bat
sed -f $HOME/git/mlr/share/src/style.sed $1 > z
mv z $1

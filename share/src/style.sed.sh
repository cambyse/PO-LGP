#cp $1 $1.bat
sed -f style.sed $1 > z
mv z $1

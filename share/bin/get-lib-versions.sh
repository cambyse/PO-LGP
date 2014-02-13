#! /bin/bash

libs=$(ldd $1 | awk '{print $3}')
packages=$(for lib in $libs; do
  name=$(dpkg -S $lib 2>/dev/null | awk '{print $1}'|sed 's/:$//')
  if [ "$name" != "" ]; then
    echo $(dpkg-query -f '${Package}-${Version}\n' -W $name)
  else
    echo "No package for library $lib found" >&2
  fi
done| sort -u)
for pkg in $packages
do
  echo $pkg
done
    



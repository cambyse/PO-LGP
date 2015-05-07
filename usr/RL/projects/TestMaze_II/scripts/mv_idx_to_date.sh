if [ $# -lt 1 ]; then
    echo "Please give file name in which tailing seconds are to be replaced date"
    exit
fi

date_sec=${1:(-14):10}
if [ ${#date_sec} -eq 0 ]; then
    echo "Length of extracted seconds is ${#date_sec} $date_sec"
    exit
elif [ $date_sec -eq $date_sec 2> /dev/null ]; then
    date_str=`date -d "@$date_sec" +%F_%T`
else
    echo "Extracted seconds are not a number, got '$date_sec'"
    exit
fi

mv -iv "$1" "${1:0:(-14)}$date_str${1:(-4):4}"

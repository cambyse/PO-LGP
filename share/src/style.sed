#spaces around keywords
s/} else {/}else{/g
s/if (/if(/g
s/for (/for(/g
s/switch (/switch(/g
s/while (/while(/g

#space between ){
s/) {/){/g

#space after comma (be careful with ',')
s/,\(\S\)/, \1/g
s/', '/','/g

#spaces around << and >>
s/\([^\s<]\)<</\1 <</g
s/<< /<</g
s/>> />>/g

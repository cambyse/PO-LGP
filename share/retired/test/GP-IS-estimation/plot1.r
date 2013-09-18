
d <- read.table('all.data');
names(d) <- c('size','obsv','simi') 

#sizes <- sort(unique(d[,'size']));
obsvs <- sort(unique(d[,'obsv']));
print(obsvs)

l=length(obsvs)
q <- array(rep(obsvs,4), dim = c(l,4))
for (i in 1:l){
	ind <- d[,'obsv']==obsvs[i];
	q[i,2:4] <- quantile( d[ind,'simi'], probs=c(.05,.5,.95) )
}

# print out for further processing by (e.g.) gnuplot
write(t(q),"",ncol=4,sep="\t")

x = q[,1]
y1 = q[,2]
y2 = q[,3]
y3 = q[,4]

xs = c(x,rev(x))
ysA = c(y1,rev(y2))
ysB = c(y2,rev(y3))
#
 plot(x,y2, type ='n',
		 main='precision of estimation: median, 5% and 95% quantile',
		 xlab='number observations', ylab='common/true volume ratio')
#
# plot(x,y2, type ='n' )
polygon(xs,ysA, col="light yellow")
polygon(xs,ysB, col="light yellow")

# see here for filled curves
# http://addictedtor.free.fr/graphiques/graphcode.php?graph=7

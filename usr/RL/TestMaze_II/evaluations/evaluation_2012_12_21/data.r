library("rgl")
library("plyr")
sparseData <- read.table("data_sparse_2x2.txt", header=TRUE)
kmdpData <- read.table("data_kmdp_2x2.txt", header=TRUE)
optimalData <- read.table("data_optimal_2x2.txt", header=TRUE)

# pick points from 'data' where 'data_check'
# matches 'data_value' in the component indicated
# by 'index' and apply 'func' to this subset
pick_simple <- function(index, func, data, data_check, data_value) {
  return( func( data[ data_check==data_value[[index]] ]))
}
pick_double <- function(index_1, index_2, func, data, data_check_1, data_value_1, data_check_2, data_value_2) {
  return( func( data[ data_check_1==data_value_1[[index_1]] & data_check_2==data_value_2[[index_2]] ]))
}

unit <- function(data) {
  return( (data-min(data))/max(data) )
}


################################################
################################################
#                                              #
#            Influence of L1-factor            #
#                                              #
################################################
################################################

## extract unique values of l1-factor and learning length
uniqueL1 <- unique(sparseData$L1)
uniqueLearn <- unique(sparseData$Learn)

### calculate mean number of features and mean-mean-reward (over learning lengths) for unique values of l1-factor
#meanFeatureNum <- sapply(1:length(uniqueL1), pick_simple, mean, sparseData$Num, sparseData$L1, uniqueL1)
#meanRew <- sapply(1:length(uniqueL1), pick_simple, mean, sparseData$MeanRew, sparseData$L1, uniqueL1)
### plot the whole thing
#open3d()
#plot3d(uniqueL1, meanRew, meanFeatureNum, xlab = "L1-Factor", ylab = "Mean Reward", zlab = "Features", type = "l", lwd = 2, box = FALSE, axes = TRUE)
#plot3d(uniqueL1, meanRew, meanFeatureNum, type = "p", size = 5, add = TRUE)
#plot3d(uniqueL1, meanRew,              0, type = "l",           add = TRUE)
#plot3d(uniqueL1, meanRew,              0, type = "p", size = 3, add = TRUE)
#plot3d(uniqueL1,       0, meanFeatureNum, type = "l",           add = TRUE)
#plot3d(uniqueL1,       0, meanFeatureNum, type = "p", size = 3, add = TRUE)
#rgl.snapshot("l1_reward_features.png","png")
#rgl.close()

###############
#   calculate mean and quartiles of number of features
#   for all combinations of unique values
#   of l1-factor and learning length
###############

prodL1Learn <- expand.grid(uniqueL1,uniqueLearn)                 # cartesian product of unique values
prodIdx <- expand.grid(1:length(uniqueL1),1:length(uniqueLearn)) # cartesian product of indices
names(prodIdx) <- c("index_1","index_2")                         # set names to function parameters

## calculate mean number of features, mean mean reward, and variance of mean reward
meanFeatureNum <- mdply( prodIdx , pick_double, mean, sparseData$Num, sparseData$L1, uniqueL1, sparseData$Learn, uniqueLearn)[,3]
meanRew <- mdply( prodIdx , pick_double, mean, sparseData$MeanRew, sparseData$L1, uniqueL1, sparseData$Learn, uniqueLearn)[,3]
meanRewVar <- mdply( prodIdx , pick_double, var, sparseData$MeanRew, sparseData$L1, uniqueL1, sparseData$Learn, uniqueLearn)[,3]

## encode value of mean reward in color
rewColor <- hsv(unit(sparseData$MeanRew)/4, 1, 1)              # for points
meanRewColor <- hsv(unit(meanRew)/4, 1-unit(meanRewVar), 1) # for mean mean reward

## plot surface of mean rewards and points that is was calculated from
persp3d(uniqueL1, uniqueLearn, meanFeatureNum, aspect=c(1, 1, 0.7), col = meanRewColor, alpha=0.8, specular="black", xlab = "", ylab = "", zlab = "")
title3d(xlab = "l1-factor", ylab = "Learning Length", zlab = "Features")
viewMatrix = t(matrix(
  c(
    0.8477355, 0.5262436, 0.0664236, 0,
    -0.2800006, 0.3376260, 0.8986701, 0,
    0.4504931, -0.7804332, 0.4335661, 0,
    0.0000000, 0.0000000, 0.0000000, 1
    )
  ,4,4))
par3d("userMatrix") # to get matrix
view3d(userMatrix=viewMatrix)

### calculate and exctract quartiles
#quantileFeatureNum <- mdply( prodIdx , pick_double, quantile, sparseData$Num, sparseData$L1, uniqueL1, sparseData$Learn, uniqueLearn)[,3:7]
#zeroQuantile <- quantileFeatureNum[,1]
#firstQuantile <- quantileFeatureNum[,2]
#secondQuantile <- quantileFeatureNum[,3]
#thirdQuantile <- quantileFeatureNum[,4]
#fourthQuantile <- quantileFeatureNum[,5]

## draw interrupted lines
meanFeatureNumNA <- c(meanFeatureNum,rep("na",length(uniqueLearn))) # append undefined values
#zeroQuantileNA <- c(zeroQuantile,rep("na",length(uniqueLearn)))     # append undefined values
#firstQuantileNA <- c(firstQuantile,rep("na",length(uniqueLearn)))   # append undefined values
#secondQuantileNA <- c(secondQuantile,rep("na",length(uniqueLearn))) # append undefined values
#thirdQuantileNA <- c(thirdQuantile,rep("na",length(uniqueLearn)))   # append undefined values
#fourthQuantileNA <- c(fourthQuantile,rep("na",length(uniqueLearn))) # append undefined values
l1 <- c(prodL1Learn[,1],rep(-0.0000000001,length(uniqueLearn)))   # negative so sorting puts it at the beginning
learn <- c(prodL1Learn[,2],uniqueLearn)                           # unique values so every line ends

#plot3d(l1[order(learn,l1)], learn[order(learn,l1)], meanFeatureNumNA[order(learn,l1)], "L1-Factor", "Learning Length", "Nr. Features", type = "n")
 
plot3d(l1[order(learn,l1)], learn[order(learn,l1)], meanFeatureNumNA[order(learn,l1)], type = "l", add = TRUE)

plot3d(sparseData$L1, sparseData$Learn, sparseData$Num, col = rewColor, size = 0.2,             type = "p", add = TRUE)
#plot3d(sparseData$L1, sparseData$Learn, sparseData$Num, col = rewColor, size = 0.2,             type = "s", add = TRUE)

#plot3d(l1[order(learn,l1)], learn[order(learn,l1)],   zeroQuantileNA[order(learn,l1)], type = "l", add = TRUE)
#plot3d(l1[order(learn,l1)], learn[order(learn,l1)],  firstQuantileNA[order(learn,l1)], type = "l", add = TRUE)
#plot3d(l1[order(learn,l1)], learn[order(learn,l1)], secondQuantileNA[order(learn,l1)], type = "l", add = TRUE)
#plot3d(l1[order(learn,l1)], learn[order(learn,l1)],  thirdQuantileNA[order(learn,l1)], type = "l", add = TRUE)
#plot3d(l1[order(learn,l1)], learn[order(learn,l1)], fourthQuantileNA[order(learn,l1)], type = "l", add = TRUE)

rgl.snapshot("plot_l1_learn_features.png","png")
rgl.close()

################################################
################################################
#                                              #
#          Comparison spars vs. kmdp           #
#                                              #
################################################
################################################

## take only entries for one (low) l1-factor
lowIdx <- 1
for(lowIdx in 1:15) {
lowRew <- sparseData$MeanRew[sparseData$L1==uniqueL1[lowIdx]]
lowLearn <- sparseData$Learn[sparseData$L1==uniqueL1[lowIdx]]

## calculate mean mean reward, mean reward variance, and quartiles

# sparse
uniqueLowLearn <- unique(lowLearn)
meanRewLow <- sapply(1:length(uniqueLowLearn), pick_simple, mean, lowRew, lowLearn, uniqueLowLearn)
RewVarLow <- sapply(1:length(uniqueLowLearn), pick_simple, var, lowRew, lowLearn, uniqueLowLearn)
quantileRewLow <- sapply(1:length(uniqueLowLearn), pick_simple, quantile, lowRew, lowLearn, uniqueLowLearn)
# kmdp
uniqueLearnKmdp <- unique(kmdpData$Lear)
meanKmdpRew <- sapply(1:length(uniqueLearnKmdp), pick_simple, mean, kmdpData$MeanRew, kmdpData$Lear, uniqueLearnKmdp)
KmdpRewVar <- sapply(1:length(uniqueLearnKmdp), pick_simple, var, kmdpData$MeanRew, kmdpData$Lear, uniqueLearnKmdp)
quantileKmdpRew <- sapply(1:length(uniqueLearnKmdp), pick_simple, quantile, kmdpData$MeanRew, kmdpData$Lear, uniqueLearnKmdp)

# optimal reward for comparison
optimalRew <- mean(optimalData[,3])
optimalRewVar <- var(optimalData[,3])
sqrt(optimalRewVar)
length(optimalData[,3])

#############
##   plot   #
#############

## set margin, title, adjust plot area
par(mar=c(7, 6, 6, 4))
#par()$mar # to see margins
subtitle <- formatC(uniqueL1[lowIdx],4,6,'f')
plot(c(kmdpData$Lear,lowLearn),c(kmdpData$MeanRew,lowRew),type="n",xlab="",ylab="")
#title(main=paste("Sparse CRF vs. k-MDP\n","l1-factor = ",subtitle,sep=""), xlab="Learning Length",ylab="Mean Reward /\nRMS Error")
title(main=paste("l1-factor = ",subtitle,sep=""))

## optimal reward
lines(uniqueLearnKmdp,rep(optimalRew-sqrt(optimalRewVar),length(uniqueLearnKmdp)),col="#0000FF",lwd=1)
lines(uniqueLearnKmdp,rep(optimalRew-sqrt(optimalRewVar),length(uniqueLearnKmdp)),col="#0000FF",lwd=1)

## sparse
points(lowLearn,lowRew,col="#99FF99",pch=4)
lines(uniqueLowLearn,meanRewLow-sqrt(RewVarLow),col="green",lwd=1)
lines(uniqueLowLearn,meanRewLow,col="green",lwd=3)
lines(uniqueLowLearn,meanRewLow+sqrt(RewVarLow),col="green",lwd=1)

## quartiles
#lines(uniqueLowLearn,quantileRewLow[2,],col="green",lwd=1)
#lines(uniqueLowLearn,quantileRewLow[3,],col="green",lwd=3)
#lines(uniqueLowLearn,quantileRewLow[4,],col="green",lwd=1)

## k-MDP
points(kmdpData$Lear,kmdpData$MeanRew,col="#FF9999",pch=4)
lines(uniqueLearnKmdp,meanKmdpRew-sqrt(KmdpRewVar),col="red",lwd=1)
lines(uniqueLearnKmdp,meanKmdpRew,col="red",lwd=3)
lines(uniqueLearnKmdp,meanKmdpRew+sqrt(KmdpRewVar),col="red",lwd=1)

## quartiles
#lines(uniqueLowLearn,quantileKmdpRew[2,],col="red",lwd=1)
#lines(uniqueLowLearn,quantileKmdpRew[3,],col="red",lwd=3)
#lines(uniqueLowLearn,quantileKmdpRew[4,],col="red",lwd=1)

dev.copy(png,paste("plot_sparse_vs_kmdp_l1_",subtitle,".png",sep=""))
dev.off()
}

graphics.off()

q()
n

install.packages("rgl")
install.packages("plyr")
library("rgl")
library("plyr")
dataSparse <- read.table("SPARSE_tmp_R.txt", header=TRUE)
dataLinearQ <- read.table("LINEAR_Q_tmp_R.txt", header=TRUE)
dataUTreeValue <- read.table("2x2_UTREE_VALUE_R.txt", header=TRUE)
dataUTreeProb <- read.table("2x2_UTREE_PROB_R.txt", header=TRUE)

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
divide_by_counts <- function(index, unique_data, var_data, count_data) {
#  return( var_data[index] / count_data[names(count_data)==unique_data[index]] )
    return( var_data[index] / count_data[names(count_data)==unique_data[index]] )
}

analyze_data <- function(file_in, file_out) {
    this_data <- read.table(file_in, header=TRUE)
    counts <- table(this_data$training_length)
    unique_values <- sort(unique(this_data$training_length))
    mean_reward <- sapply(1:length(unique_values), pick_simple, mean, this_data$episode_mean_reward, this_data$training_length, unique_values)
    reward_variance <- sapply(1:length(unique_values), pick_simple, var, this_data$episode_mean_reward, this_data$training_length, unique_values)
    mean_variance <- sapply(1:length(unique_values), divide_by_counts, unique_values, reward_variance, counts)
    out <- cbind(unique_values,mean_reward,as.vector(mean_variance))
    write(t(out), file_out, ncolumns=3, sep="	")
}

analyze_data("LINEAR_Q_tmp_R.txt","out_data.txt");



sparse_counts <- table(dataSparse$training_length)
sparse_unique_values <- sort(unique(dataSparse$training_length))
sparse_mean_reward <- sapply(1:length(sparse_unique_values), pick_simple, mean, dataSparse$episode_mean_reward, dataSparse$training_length, sparse_unique_values)
sparse_reward_variance <- sapply(1:length(sparse_unique_values), pick_simple, var, dataSparse$episode_mean_reward, dataSparse$training_length, sparse_unique_values)
sparse_mean_variance <- sapply(1:length(sparse_unique_values), divide_by_counts, sparse_unique_values, sparse_reward_variance, sparse_counts)

sparse_out <- cbind(sparse_unique_values,sparse_mean_reward,as.vector(sparse_mean_variance))

write(t(sparse_out), "out_data.txt", ncolumns=3, sep="	")

plot(sparse_unique_values, sparse_mean_reward)
points(sparse_unique_values, sparse_mean_reward+sqrt(sparse_mean_variance))
points(sparse_unique_values, sparse_mean_reward-sqrt(sparse_mean_variance))

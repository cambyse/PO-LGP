#install.packages("rgl")
#install.packages("plyr")
library("rgl")
library("plyr")

# pick points from 'data' where 'data_check'
# matches 'data_value' in the component indicated
# by 'index' and apply 'func' to this subset
pick_matching <- function(index, func, data, data_check, data_value) {
  return( func( data[ data_check==data_value[[index]] ]))
}

# divide
divide_by_counts <- function(index, unique_data, var_data, count_data) {
    return( var_data[index] / count_data[names(count_data)==unique_data[index]] )
}

analyze_data <- function(file_in, file_out) {
    this_data <- read.table(file_in, header=TRUE)
    counts <- table(this_data$training_length)
    unique_values <- sort(unique(this_data$training_length))
    mean_reward <- sapply(1:length(unique_values), pick_matching, mean, this_data$episode_mean_reward, this_data$training_length, unique_values)
    reward_variance <- sapply(1:length(unique_values), pick_matching, var, this_data$episode_mean_reward, this_data$training_length, unique_values)
    mean_variance <- sapply(1:length(unique_values), divide_by_counts, unique_values, reward_variance, counts)
    mean_variance <- sqrt(mean_variance)
    out <- cbind(unique_values,mean_reward,as.vector(mean_variance),as.vector(counts))
    write(t(out), file_out, ncolumns=4, sep="	")
}

analyze_data("2x2_SPARSE_R.txt","2x2_SPARSE_plot.txt");
analyze_data("2x2_LINEAR_Q_R.txt","2x2_LINEAR_Q_plot.txt");
analyze_data("2x2_UTREE_VALUE_R.txt","2x2_UTREE_VALUE_plot.txt");
analyze_data("2x2_UTREE_PROB_R.txt","2x2_UTREE_PROB_plot.txt");

analyze_data("4x4_III_SPARSE_R.txt","4x4_III_SPARSE_plot.txt");
analyze_data("4x4_III_UTREE_VALUE_R.txt","4x4_III_UTREE_VALUE_plot.txt");

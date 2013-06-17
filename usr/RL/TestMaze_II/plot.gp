rand_fact = 0
rand_fact_x = rand_fact*200
rand_fact_y = rand_fact*0.005

point_type = 6

plot [0:][0:0.3] \
     'log_file_linear-q_1371244476.txt' u ($2+rand_fact_x*rand(0)):($5+rand_fact_y*rand(0)) s u w lp, \
     'log_file_sparse_1371244363.txt' u ($2+rand_fact_x*rand(0)):($5+rand_fact_y*rand(0)) s u w lp, \
     'log_file_utree-prob_1371244416.txt' u ($2+rand_fact_x*rand(0)):($5+rand_fact_y*rand(0)) s u w lp, \
     'log_file_utree-value_1371244456.txt' u ($2+rand_fact_x*rand(0)):($5+rand_fact_y*rand(0)) s u w lp
#     'log_file_random_1371490133.txt' u ($2+rand_fact_x*rand(0)):($5+rand_fact_y*rand(0)) s u w lp

plot [0:][0:0.3] \
     'log_file_linear-q_1371244476.txt' u ($2+rand_fact_x*rand(0)):($5+rand_fact_y*rand(0)) pt point_type , \
     'log_file_sparse_1371244363.txt' u ($2+rand_fact_x*rand(0)):($5+rand_fact_y*rand(0)) pt point_type , \
     'log_file_utree-prob_1371244416.txt' u ($2+rand_fact_x*rand(0)):($5+rand_fact_y*rand(0)) pt point_type , \
     'log_file_utree-value_1371244456.txt' u ($2+rand_fact_x*rand(0)):($5+rand_fact_y*rand(0)) pt point_type
#     'log_file_random_1371490133.txt' u ($2+rand_fact_x*rand(0)):($5+rand_fact_y*rand(0)) pt point_type

plot [0:][0:0.3] 'log_file_linear-q_1371244476.txt' u ($2+rand_fact_x*rand(0)):($5+rand_fact_y*rand(0)) pt point_type
plot [0:][0:0.3] 'log_file_sparse_1371244363.txt' u ($2+rand_fact_x*rand(0)):($5+rand_fact_y*rand(0)) pt point_type
plot [0:][0:0.3] 'log_file_utree-prob_1371244416.txt' u ($2+rand_fact_x*rand(0)):($5+rand_fact_y*rand(0)) pt point_type
plot [0:][0:0.3] 'log_file_utree-value_1371244456.txt' u ($2+rand_fact_x*rand(0)):($5+rand_fact_y*rand(0)) pt point_type
#plot [0:][0:0.3] 'log_file_random_1371490133.txt' u ($2+rand_fact_x*rand(0)):($5+rand_fact_y*rand(0)) pt point_type
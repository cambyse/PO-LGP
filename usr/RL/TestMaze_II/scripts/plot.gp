rand_fact = 0
rand_fact_x = rand_fact*1
rand_fact_y = rand_fact*0.002
r_x(x)=rand_fact_x*(2*rand(0)-1)
r_y(y)=rand_fact_y*(2*rand(0)-1)

point_type = 6

plot [0:10000][0:0.3] \
     '../log_file_linear-q_2013-06-14_23:14:36.txt' u ($2+r_x(0)):($5+r_y(0)) s u w lp t 'linear-q', \
     '../log_file_sparse_1371244363.txt' u ($2+r_x(0)):($5+r_y(0)) s u w lp t 'sparse', \
     '../log_file_utree-prob_2013-06-14_23:13:36.txt' u ($2+r_x(0)):($5+r_y(0)) s u w lp t 'u-prob', \
     '../log_file_utree-value_2013-06-14_23:14:16.txt' u ($2+r_x(0)):($5+r_y(0)) s u w lp t 'u-value'
#     '../log_file_random_1371490133.txt' u ($2+r_x(0)):($5+r_y(0)) s u w lp

plot [0:][0:0.3] \
     '../log_file_linear-q_2013-06-14_23:14:36.txt' u ($2+r_x(0)):($5+r_y(0)) pt point_type t 'linear-q', \
     '../log_file_sparse_1371244363.txt' u ($2+r_x(0)):($5+r_y(0)) pt point_type t 'sparse' , \
     '../log_file_utree-prob_2013-06-14_23:13:36.txt' u ($2+r_x(0)):($5+r_y(0)) pt point_type t 'u-prob' , \
     '../log_file_utree-value_2013-06-14_23:14:16.txt' u ($2+r_x(0)):($5+r_y(0)) pt point_type t 'u-value'
#     '../log_file_random_1371490133.txt' u ($2+r_x(0)):($5+r_y(0)) pt point_type

plot [0:][0:0.3] '../log_file_linear-q_2013-06-14_23:14:36.txt' u ($2+r_x(0)):($5+r_y(0)) pt point_type t 'linear-q'
plot [0:][0:0.3] '../log_file_sparse_1371244363.txt' u ($2+r_x(0)):($5+r_y(0)) pt point_type t 'sparse'
plot [0:][0:0.3] '../log_file_utree-prob_2013-06-14_23:13:36.txt' u ($2+r_x(0)):($5+r_y(0)) pt point_type t 'u-prob'
plot [0:][0:0.3] '../log_file_utree-value_2013-06-14_23:14:16.txt' u ($2+r_x(0)):($5+r_y(0)) pt point_type t 'u-value'
#plot [0:][0:0.3] '../log_file_random_1371490133.txt' u ($2+r_x(0)):($5+r_y(0)) pt point_type
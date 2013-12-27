pkg load control

A=[
 0 0 1 0;
  0 0 0 1;
  0 -57.6471 0 0;
  0 129.706 0 0 ]
B=[
 0;
  0;
  268.235;
  -523.529 ]

Q = diag([1 0 1 0]) #position costs
R = diag([100])      #control costs

P = care(A,B,Q,R)

K = - inverse(R)*B'*P

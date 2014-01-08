pkg load control
A=[
 0 0 1 0
  0 0 0 1
  0 -13.2757 0 0
  0 43.3127 0 0 ]
B=[
 0
  0
  32.3676
  -64.5758 ]

Q = diag([10 0 1 0]) #position costs
R = diag([1])       #control costs

P = care(A,B,Q,R)

K = - inverse(R)*B'*P

pkg load control

g=9.8
l=1
Mp=1
Mc=1
c1=1/(Mp+Mc)
c2=l*Mp*c1

A=[
0 1 0 0
0 0 -c2*g/((4/3)*l-c2) 0
0 0 0 1
0 0 g/((4/3)*l-c2) 0
]

B=[
0
c1+c1*c2/((4/3)*l-c2)
0
-c1/((4/3)*l-c2)
]

%A=[
% 0 0 1 0
% 0 0 0 1
% 0 -13.2757 0 0
% 0 43.3127 0 0 ]
%B=[
% 0
%  0
%  32.3676
%  -64.5758 ]

Q = diag([1 0 1 0]) #position costs
R = diag([1])       #control costs

P = care(A,B,Q,R)

K = - inverse(R)*B'*P

pkg load control

l=1
g=9.81
c1=1/2
c2=1/2

B = [0 c1 0 -c1/(4/3*l-c2)]'
A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 g/(4/3*l-c2) 0]

ex=1
eth=1
rho=1

Q = diag([ex 0 eth 0])
R = rho

P = are(A,B*inverse(R)*B',Q)

K = - inverse(R)*B'*P
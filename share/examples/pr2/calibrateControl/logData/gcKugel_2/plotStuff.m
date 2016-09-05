q = load('q');
u = load('u');

qSign = load('qSign');

u = u(:,9);
q = q(:,9);
qSign = qSign(:,9);


plot(q(qSign < 0),u(qSign < 0),'.r')
hold on;
plot(q(qSign > 0),u(qSign > 0),'.b')
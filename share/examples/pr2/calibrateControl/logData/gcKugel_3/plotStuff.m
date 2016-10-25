q = load('q');
u = load('u');

qSign = load('qSign');

u = u(:,7);
q = q(:,7);
qSign = qSign(:,7);


plot(q(qSign < 0),u(qSign < 0),'.r')
hold on;
plot(q(qSign > 0),u(qSign > 0),'.b')
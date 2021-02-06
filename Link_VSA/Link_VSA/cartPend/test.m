
syms x1 x2 dx1 dx2 m l g
g1= -m*l*(dx2^2)*sin(x2);
g2= -g* sin(x2);

g=[g1;g2];

gg=jacobian(g,[x1, x2])
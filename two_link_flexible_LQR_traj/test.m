% Normal dynamics is: 
% 
% H*ddq + c = u
% 
% SEA dynamics is:
% 
% H1*ddq + c = K*(theta - q)
% H2*ddtheta = u - K*(theta - q)
% 
% VSA dynamics is only different in that K is not a constant

m = [5; 5];
L = [1; 1];
g = 9.81;

I = [(1/12)*m(1)*L(1)^2; (1/12)*m(2)*L(2)^2];

q = sym('q', [2, 1]); assume(q, 'real');
v = sym('v', [2, 1]); assume(v, 'real');

Math = MathClass;
Math.UseParallel = false;

rC1 = [0.5*L(1)*cos(q(1)); 0.5*L(1)*sin(q(1))];
rO1 = [    L(1)*cos(q(1));     L(1)*sin(q(1))];

rC2 = rO1 + [0.5*L(2)*cos(q(2)); 
             0.5*L(2)*sin( q(2))
           ];
rO2 = rO1 + [L(2)*cos(q(2)); 
             L(2)*sin(q(2))
             ];
         
JC1 = jacobian(rC1, q);

JC2 = jacobian(rC2, q);

JC1 = simplify(JC1);
JC2 = simplify(JC2);

Je = jacobian(rO2,q)

Jedot = diff(Je)

subs(Jedot, {q(1), q(2)},{pi/6, pi/6})


G = [Je zeros(2,2);
    zeros(2,4);
    Jedot  Je;
    zeros(2,4)]

con=subs(G, {q(1), q(2)},{pi/6, pi/6});
A= rand(4,4);
B = rand(4,1);
N=null(con);

An = N'* A*N


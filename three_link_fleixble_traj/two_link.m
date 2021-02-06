clc; clear; close all;

% syms l1 l2 m1 m2 g real

m1 = 1;
m2= 1;
g= 10;
l1= 1;
l2= 1;

m = [m1; m2];
L = [l1; l2];

% g = 9.81;
% note to self
% remember to replace all m1 m2 l1 l2 and Inertai by their numeric value
I = [(1/12)*m(1)*L(1)^2; (1/12)*m(2)*L(2)^2];

% I1= I(1)



q = sym('q', [2, 1]); assume(q, 'real');

v = sym('v', [2, 1]); assume(v, 'real');

p1 = [0.5*L(1)* cos(q(1));
    0.5*L(1)* sin(q(1));
    0];
         
p2 = [L(1)* cos(q(1))+0.5*L(2) * cos(q(2)+q(1));
    L(1)* sin(q(1))+0.5*L(2) * sin(q(2)+q(1));
    0];

JC1 = simplify(jacobian(p1, q));
JC2 = simplify(jacobian(p2, q));

omega1 = [0; 0; v(1)];
omega2 = [0; 0; v(1)+v(2)];

Jw1 = jacobian(omega1, v);
Jw2 = jacobian(omega2, v);

H = JC1' * m(1) * JC1 + ...
    JC2' * m(2) * JC2 + ...
    Jw1' * I(1) * Jw1 + ...
    Jw2' * I(2) * Jw2;

M= simplify(H);

b1 = M(1,:)';
b2 = M(2,:)';

diff_b1= jacobian(b1,q);
diff_b2= jacobian(b2,q);

diff_B1 = diff(M, q(1));
diff_B2 = diff(M, q(2));

c1 = 0.5*(diff_b1 + diff_b1'-diff_B1);
c2 = 0.5*(diff_b2 + diff_b2'-diff_B2);
C1 = v' * c1 *v;
C2 = v' * c2 *v;

C = [C1; C2];

V = m1 * g* L(1)/2*sin(q(1))+ m2* g* (L(1)* sin(q(1))+ L(2)/2* sin(q(2)));

G = [diff(V, q(1)); diff(V, q(2))];

% Jc = simplify(jaco)

u = sym('u', [2, 1]); assume(u, 'real');

H=[1 0; 0 1];
beta = C+G;
ddq = M\(H*u - beta);
ddq= simplify(ddq);
theta4=m2*l1+ m1*l1*0.5;
theta5=m2*l2*0.5;
energy = v'*M*v + G;

% var =[g;l1;l2;m1;m2]
s = [q;v];
ds = [v;ddq];

A = jacobian(ds, s);
A = simplify(A);
size(A)

linA= subs(A, [q(1); q(2); v(1); v(2)], [pi/2; pi/2; 0; 0]);

B = jacobian(ds, u);

C = ddq - A*x - B*u;
C = simplify(C); 


simplify(A);
simplify(B);

linA= subs(A,[q(1), q(2), v(1), v(2)],[pi/2,pi/2,0,0]);
linB= subs(B,[q(1), q(2), v(1), v(2)],[pi/2,pi/2,0,0]);

matlabFunction(ds, 'File', 'dynamics', 'Vars', {s,u});
matlabFunction(M, 'File', 'inertia', 'Vars', {s,u});
matlabFunction(energy, 'File', 'lyapunov', 'Vars', {s});

matlabFunction(A, 'File', 'g_Linear_A', 'Vars', {s,u});
matlabFunction(B, 'File', 'g_Linear_B', 'Vars', {s,u});

matlabFunction(C, 'File', 'g_Linear_C', 'Vars', {x});
matlabFunction(ddq, 'File', 'g_LHS', 'Vars', {x, u});

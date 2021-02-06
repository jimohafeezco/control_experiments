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


rC1 = [0.5*L(1)*cos(q(1)); 0.5*L(1)*sin(q(1)); 0];
rO1 = [    L(1)*cos(q(1));     L(1)*sin(q(1)); 0];

rC2 = rO1 + [0.5*L(2)*cos(q(2)); 
             0.5*L(2)*sin( q(2));
             0];
rO2 = rO1 + [L(2)*cos(q(2)); 
             L(2)*sin(q(2));
             0];
         
         
JC1 = jacobian(rC1, q);

JC2 = jacobian(rC2, q);

JC1 = simplify(JC1);
JC2 = simplify(JC2);


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


c1 = 0.5*(jacobian(H(:,1),q)+jacobian(H(:,1),q)'- diff(H,q(1)));
c2 = 0.5*( jacobian(H(:,2),q)+jacobian(H(:,2),q)'- diff(H,q(2)));

beta= [v'*c1*v; v'*c2*v];
% Jc = simplify(jaco)

u = sym('u', [2, 1]); assume(u, 'real');

H=[1 0; 0 1];
% beta = C+G;
ddq = M\(H*u - beta);
ddq= simplify(ddq);

s = [q;v];
ds = [v;ddq];

A = jacobian(ds, s);
A = simplify(A);

A= subs(A, {u(1),u(2)},{0,0});

B = jacobian(ds, u);
% 
C = ds - A*s - B*u;
C = simplify(C); 
% 

Je = jacobian(rO2,q);

Jedot = diff(Je);


G = [Je zeros(2,2);
    Jedot  Je];
matlabFunction(G, 'File', 'g_constraint', 'Vars', {x});


% 
% 
matlabFunction(A, 'File', 'g_Linear_A', 'Vars', {s});
matlabFunction(B, 'File', 'g_Linear_B', 'Vars', {s});
% 
matlabFunction(C, 'File', 'g_Linear_C', 'Vars', {s,u});
% matlabFunction(ddq, 'File', 'g_LHS', 'Vars', {x, u});
matlabFunction(ds, 'File', 'dynamics', 'Vars', {s,u});
matlabFunction(rO2, 'File', 'g_rO2', 'Vars', {q});


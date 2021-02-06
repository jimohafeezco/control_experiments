% syms l1 l2 l3 m1 m2 m3 g I1 I2 I3 real
clear;

m1=1;
m2=1;
m3=1;
l1=1;
l2=1;
l3=1;
g=9.81;
L = [l1; l2; l3];
m = [m1; m2; m3];

% g = 9.81;
% note to self
% remember to replace all m1 m2 l1 l2 and Inertai by their numeric value
I = [(1/12)*m(1)*L(1)^2; (1/12)*m(2)*L(2)^2; (1/12)*m(3)*L(3)^2];
% I= [I1; I2; I3];
q = sym('q', [3, 1]); assume(q, 'real');
qs = sym('qs', [3, 1]); assume(qs, 'real');

v = sym('dq', [3, 1]); assume(v, 'real');
vs = sym('dqs', [3, 1]); assume(vs, 'real');

u = sym('u', [3, 1]); assume(u, 'real');


p1 = [0.5*L(1)* cos(q(1));
    0.5*L(1)* sin(q(1));
    0];
         
p2 = [L(1)* cos(q(1))+0.5*L(2) * cos(q(1)+q(2));
    L(1)* sin(q(1))+0.5*L(2) * sin(q(1)+q(2));
    0];

p3 = [L(1)* cos(q(1))+L(2) * cos(q(1)+q(2))+0.5*L(3) * cos(q(1)+q(2)+ q(3));
    L(1)* sin(q(1))+L(2) * sin(q(1)+q(2))+ 0.5*L(3) * sin(q(1)+q(2)+q(3));
    0];

JC1 = simplify(jacobian(p1, q));
JC2 = simplify(jacobian(p2, q));
JC3 = simplify(jacobian(p3, q));

omega1 = [0; 0; v(1)];
omega2 = [0; 0; v(1)+v(2)];
omega3 = [0; 0; v(1)+v(2)+v(3)];

Jw1 = jacobian(omega1, v);
Jw2 = jacobian(omega2, v);
Jw3 = jacobian(omega3, v);

H = JC1' * m(1) * JC1 + ...
    JC2' * m(2) * JC2 + ...
    JC3' * m(3) * JC3 + ...
    Jw1' * I(1) * Jw1 + ...
    Jw2' * I(2) * Jw2 + ...
    Jw3' * I(3) * Jw3;

B = simplify(H);

b1 = B(1,:)';
b2 = B(2,:)';
b3 = B(3,:)';

diff_b1= jacobian(b1,q);
diff_b2= jacobian(b2,q);
diff_b3= jacobian(b3,q);

diff_B1 = diff(B, q(1));
diff_B2 = diff(B, q(2));
diff_B3 = diff(B, q(3));

c1 = 0.5*(diff_b1 + diff_b1'-diff_B1);
c2 = 0.5*(diff_b2 + diff_b2'-diff_B2);
c3 = 0.5*(diff_b3 + diff_b3'-diff_B3);

C1 = v' * c1 *v;
C2 = v' * c2 *v;
C3 = v' * c3 *v;

C = [C1; C2; C3];

V = m1 * g* L(1)/2*sin(q(1))+ ...
    m2* g* (L(1)* sin(q(1))+ L(2)/2* sin(q(1)+q(2)))+...
    m3* g* (L(1)* sin(q(1))+ L(2)* sin(q(1)+q(2))+ 0.5* L(3)* sin(q(1)+q(2)+q(3)));
  
G = [diff(V, q(1));...
    diff(V, q(2));...
    diff(V, q(3))];




B1 = eye(3)*1;
D = blkdiag(B, B1);
Binv= inv(B);
% k=1;
K = diag([1000 1000 1000]);
% B= diag([2 2]);
% damp= B*[x(3); x(4)] 

F_elastic = K*[qs-q];

beta  =[C+G-F_elastic;
        F_elastic];

% beta = C+G;

H2=[0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1];

ddq = D\(H2*u - beta);
ddq= simplify(ddq);

% x=[x(1), x(2), x(3), x(4), dx(1), dx(2), dx(3), dx(4)];
s = [q;qs;v; vs];
ds = [v;vs; ddq];

A = jacobian(ds, s);
% A = simplify(A);
% size(A)

B = jacobian(ds, u);
% B = simplify(B);
matlabFunction(ds, 'File', 'dynamics', 'Vars', {s,u});
matlabFunction(D, 'File', 'inertia', 'Vars', {s});
matlabFunction(beta, 'File', 'beta', 'Vars', {s});

matlabFunction(A, 'File', 'g_Linear_A', 'Vars', {s,u});
matlabFunction(B, 'File', 'g_Linear_B', 'Vars', {s,u});
% eye(2)*0.1
%  
C = ds - A*s - B*u;
% % C = simplify(C); 
matlabFunction(C, 'File', 'g_Linear_C', 'Vars', {s,u});

% u0 = pinv(B) * (dx - A*x - C);

% Jc = simplify(jaco)
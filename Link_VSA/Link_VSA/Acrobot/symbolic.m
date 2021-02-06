clc; clear; close all;

x = sym('x', [2, 1]); assume(x, 'real');
dx = sym('dx', [2, 1]); assume(dx, 'real');

m1=1;
m2=1;
l1=0.5;
l2=0.5;
lc1=0.5*l1;
lc2=0.5*l2;

g=9.81;
I1=0.008;
I2=0.33;


c1=m1*(lc1^2)+m2*l1^2+I1;
c2=m2*lc2^2+I2;
c3=m2*l1*lc2;

d11 = c1+c2+2*c3*cos(x(2));
d12= c2+c3*cos(x(2));
d21=d12;
d22=c2;

h=-m2*l1* 0.5* l2* sin(x(2));
c11=h *dx(2);
c12=h *dx(2)+ h*dx(1);
c21= -h *dx(1);
c22= 0;
g1= (m1* 0.5* l1 + m2* l1)* g * cos(x(1))+m2 * (0.5*l2)* g * cos(x(1)+x(2));
g2 =m2 * (0.5*l2)* g * cos(x(1)+x(2));
% Find 'Inertia matrix'
D = [d11 d12;
    d21 d22];

G=[g1;g2];
C=[c11+c12; c21+c22];

u = sym('u', [1, 1]); assume(u, 'real');
% % 
k = sym('k', [2, 1]); assume(k, 'real');
% % 
% % 
% D2 = eye(2)*0.01;
% 
% 
% u_springs = [k(1)*(q_theta(1) - x(1));
%              k(1)*(x(1)+q_theta(2) - x(2))];
% 
% k=1;
% K = diag([5 5]);
% LHS = [K*(q_theta - q) - C;
%        u(1:2) - K*(q_theta - q)];
%    

% F_elastic = K*[(x(3)-x(1));
%                (x(4)-x(2))];

beta  =C+G;
    
% D = blkdiag(D, D2);
H=[0 ;1];

ddx = D\(H*u - beta)
ddx= simplify(ddx);

% x=[x(1), x(2), x(3), x(4), dx(1), dx(2), dx(3), dx(4)];
s = [x;dx];
ds = [dx; ddx];
A = jacobian(ds, s);
A = simplify(A);
% size(A)

B = jacobian(ds, u);
B = simplify(B);
matlabFunction(ds, 'File', 'dynamic', 'Vars', {s,u});

matlabFunction(A, 'File', 'g_Linear_A', 'Vars', {s,u});
matlabFunction(B, 'File', 'g_Linear_B', 'Vars', {s,u});
% eye(2)*0.1
 
C = ds - A*s - B*u;
C = simplify(C); 
matlabFunction(C, 'File', 'g_Linear_C', 'Vars', {s,u});

% u0 = pinv(B) * (dx - A*x - C);

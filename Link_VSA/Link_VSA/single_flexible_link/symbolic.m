 clc; clear; close all;

x = sym('x', [2, 1]); assume(x, 'real');
dx = sym('dx', [2, 1]); assume(dx, 'real');

m=1;
l=1;
% lc1=0.5*l1;
B=0.05;
g=9.81;
I=0.1;
K =1000;
J=0.1;
% Find 'Inertia matrix'
C= m*g*l* sin(x(1))
F_elastic = K*(x(1)-x(2));


beta  =[C+F_elastic;
        B*dx(2)-F_elastic];
    
u = sym('u', [1, 1]); assume(u, 'real');
% % 
k = sym('k', [2, 1]); assume(k, 'real');
% % 
% % 
D=[I 0;
    0 J]

% K = diag([1 1]);

   


% D = blkdiag(D, D2);
H=[0 1]';

ddx = D\(H*u - beta);
ddx= simplify(ddx);
% 
% % x=[x(1), x(2), x(3), x(4), dx(1), dx(2), dx(3), dx(4)];
s = [x;dx];
ds = [dx; ddx];
A = jacobian(ds, s);
A = simplify(A);
% % size(A)
% 
B = jacobian(ds, u);
B = simplify(B);
matlabFunction(ds, 'File', 'dynamic', 'Vars', {s,u});
% 
matlabFunction(A, 'File', 'g_Linear_A', 'Vars', {s});
matlabFunction(B, 'File', 'g_Linear_B', 'Vars', {s});
matlabFunction(beta, 'File', 'g_beta', 'Vars', {s});
% matlabFunction(D, 'File', 'D_sys', 'Vars', {s});
% 
 
C = ds - A*s - B*u;
C = simplify(C); 
matlabFunction(C, 'File', 'g_Linear_C', 'Vars', {s});

% u0 = pinv(B) * (dx - A*x - C);

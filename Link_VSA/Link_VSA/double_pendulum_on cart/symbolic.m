function dstate =symbolic()

clc; clear; close all;
m1 = 1.5;m2=1.5; m3=0.5;
l1= 1; l2 =1; l3=1;

system.l1 =l1;
system.l2=l2;
system.l3=l3;
system.m1=m1;
system.m2=m2;
system.m3=m3;

% k=system.b;
dstate =dynamics(system)
function ds= dynamics(system)

u = sym('u', [1, 1]); assume(u, 'real');

x = sym('x', [3, 1]); assume(x, 'real');
dx = sym('dx', [3, 1]); assume(dx, 'real');
l1 = system.l1;
l2= system.l2;
l3= system.l3;
m1 = system.m1;
m2= system.m2;
m3= system.m3;
% b= system.b;
% k=system.b;
% Extract states 
d1 = m1+m2+m3;
d2= (0.5*m1+m2)*l1;
d3= 0.5*m2*l2;
d4= (1/3*m1+m2)*l1^2;
d5=0.5*m2*l1*l2;
f1= ((0.5*m1)+m2)*l1*10;
f2= 0.5*m2*l2*10;
d6= 1/3*m2*l2^2;


D = [d1,   d2,    d3;
    d2,    d4,     d5;
    d3,      d5,    d6];


C1 = [0  -d2* x(2) -d3*x(3);
    0   0   d5* (x(2)-x(3));
    0   -d5    0];
C= C1*[dx(1); dx(2); dx(3)];
G= [0 ;
    -f1*x(2);
    -f2* x(3)];
beta= C+G;
% u_0 = [a*(3+dx(1^2), 1; a, 1+dx2^2];
H = [1 0 0]';

% Equation for second order derevitive
ddx = D\(H*u - beta);
ddx= simplify(ddx);

s = [x;dx];

ds = [dx; ddx]

A = jacobian(ds, s);
A = simplify(A);
% size(A)

B = jacobian(ds, u);
B = simplify(B);
matlabFunction(ds, 'File', 'dynamic', 'Vars', {s,u});

matlabFunction(A, 'File', 'g_Linear_A', 'Vars', {s});
matlabFunction(B, 'File', 'g_Linear_B', 'Vars', {s});
% eye(2)*0.1
 
C = ds - A*s - B*u;
C = simplify(C); 
matlabFunction(C, 'File', 'g_Linear_C', 'Vars', {s,u});

% ds = [dx; ddx];

end

end
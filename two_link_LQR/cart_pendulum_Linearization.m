clc; clear; close all;

syms l m1 m2 g I1 I2 q dq x dx  u real
% q = sym('q', [1, 1]); assume(q, 'real');

M= [cos(q), l;
    m1+m2 m2*l*cos(q)];

h = [-g*sin(q);
    u+ m2*l*dq^2* sin(q)];

ddq = simplify(M\h);
% 
ds =[dx; dq;ddq(1); ddq(2)];
% s=[]
s = [x; q;dx;dq]
A = jacobian(ds, s);
% 
B = jacobian(ds, u);

subs(A,s,[0;0; 0;0])
subs(B,s,[0;0; 0;0])



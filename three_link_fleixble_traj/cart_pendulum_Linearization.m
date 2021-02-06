clc; clear; close all;

syms l m1 m2 g I1 I2 q dq x dx  u real
% q = sym('q', [1, 1]); assume(q, 'real');

M= [cos(q), l;
    m1+m2 m2*l*cos(q)];

h = [-g*sin(q);
    u+ m2*l*dq^2* sin(q)];

ddq = simplify(M\h);
% 
ds =[dx; ddq(1);dq; ddq(2)];
% s=[]
s = [x; dx; q;dq]
A = jacobian(ds, s)
% 
B = jacobian(ds, u)




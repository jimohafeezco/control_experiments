function [u, x_des] = control(state)
x1 = state(1); 
x2 = state(2); 


x=[x1; x2];
dx1 = state(3); 
dx2 = state(4); 
dx = [dx1; dx2];

s=[x; dx];
x_des=[pi; pi; 0 ; 0];
A = g_Linear_A(s);
% Q = 1000*diag([0.05; 0.05; 0.01; 0.01; 1; 1; 1; 1]);
Q= eye(4)*100;
R = 0.1;
B = g_Linear_B(s);
K = lqr(A, B, Q, R);

rank(ctrb(A,B));
% 
C = g_Linear_C(s);
% 
u0 = pinv(B) * (-A*s - C);
% 
e=s-x_des;
u= -K*e+u0;
% u=[0; 0];
% u=-K* e

end
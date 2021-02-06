function [u, des] = control(state)

q1 = state(1); 
q2 = state(2); 
q3 = state(3); 

q1s = state(4); 
q2s = state(5); 
q3s = state(6);

q=[q1; q2; q3; q1s; q2s; q3s];

dq1 = state(7); 
dq2 = state(8); 
dq3 = state(9); 
dq1s = state(10); 
dq2s = state(11); 
dq3s = state(12); 
dq = [dq1; dq2; dq3; dq1s; dq2s; dq3s];

x = [q; dq];


% u1 = [1;1;1];

x_des = [pi/2; pi/2; pi/2;pi/2; pi/2; pi/2]; 
dx_des = [0 ; 0; 0; 0 ; 0; 0];

des = [x_des; dx_des];



A = g_Linear_A(x);
% Q = 1000*diag([0.05; 0.05; 0.01; 0.01; 1; 1; 1; 1]);
Q= eye(12)*1000;
R = eye(3)*0.001;
B = g_Linear_B(x);
K = lqr(A, B, Q, R);


C = g_Linear_C(x);

u0 = pinv(B) * (-A*x - C);

e=x-des;
u= -K*e+u0;


end
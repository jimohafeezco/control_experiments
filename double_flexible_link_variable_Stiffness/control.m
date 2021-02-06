% a= rand(8,1)
function [u, x, x_des] = control(state)
x1 = state(1); 
x2 = state(2); 
x3 = state(3); 
x4 = state(4); 
x5 = state(5); 
x6 = state(6); 


x=[x1; x2; x3; x4; x5; x6];
dx1 = state(7); 
dx2 = state(8); 
dx3 =    state(9); 
dx4 = state(10); 
dx5 =    state(11); 
dx6 = state(12); 
dx = [dx1; dx2; dx3; dx4; dx5; dx6];

s=[x; dx];
% x_des=controller.x_des;
% s=[x(1);x(2);x(3); x(4); dx(1);dx(2); dx(3); dx(4)];
% u1 = 1;
x_des=[pi/2; pi/2 ; pi/2;pi/2; 0 ; 0; 0 ; 0];
% bb=[x_des1; zeros(4,1)];

% thetades= g_beta(bb)+x_des1;

% x_des=[x_des1; thetades];
A = g_Linear_A(s)
% Q = 1000*diag([0.05; 0.05; 0.01; 0.01; 1; 1; 1; 1]);
Q= eye(12)*1000;
R = eye(2)*0.001;
B = g_Linear_B(s);
K = lqr(A, B, Q, R);

rank(ctrb(A,B));

C = g_Linear_C(s);

u0 = pinv(B) * (-A*s - C);

e=s-x_des;
u= -[K(1:4);K(6:10)]*e+u0;
% u=[0; 0];


end


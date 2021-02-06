function [u, des] = control(state)

q1 = state(1); 
q2 = state(2); 
q3 = state(3); 

q=[q1; q2; q3];

dq1 = state(4); 
dq2 = state(5); 
dq3 = state(6); 

dq = [dq1; dq2; dq3];

x = [q; dq];


% u1 = [1;1;1];

x_des = [pi/2; pi/3; pi/4]; 
dx_des = [0 ; 0; 0];

des = [x_des; dx_des];

kp = eye(3)*36;
kd = eye(3)*12;


u_star = kp *(x_des-q)+ kd*(dx_des-dq);
D = inertia(x);
bet = beta(x);
% size(D)
% size(bet)
% size(u_star)
u = D*u_star+ bet;
end
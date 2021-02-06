function [Jdot] = cost(state, linear)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

x1 = state(1); 
x2 = state(2); 
x3 = state(3); 
x4 = state(4); 

dx1 = state(5); 
dx2 = state(6); 
dx3 =    state(7); 
dx4 = state(8); 

x=[x1; x2; x3; x4; dx1; dx2; dx3; dx4];
% x = state;
Q =linear.Q;
R =linear.R;

[u,~, x_des]= control(state, linear);
% k=0.1;
Jdot = 0.5*exp(2*0.1)*((x_des-x)'* Q *(x_des-x) + u'* R*u);


end


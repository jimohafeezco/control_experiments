%single pendulum on cart
clear; clc; close all;clf;
% x_des=[pi/2; pi; pi/2; pi/2; 0 ; 0; 0 ; 0];

q0= [pi/2-pi/9; 0.5]; 
qdot0=[0; 0];
% x_des=[0; 0; pi; pi; 0 ; 0; 0 ; 0];
x0=[q0;qdot0];
% Time settings
t0 = 0;
tf = 10;
dt = 0.001;

% Integration (simulation)
% Create a time vector for numerical integration

tspan = [t0:dt:tf]; 

% [t,x] = ode45(@(t,x)dynamic(x, control(x)), tspan, x0)

for i=1:length(tspan)
    x0=[q0;qdot0];
    qdotdot=dynamics(x0);
    qdot= qdot0+dt*qdotdot;
    q=q0+dt*qdot+0.5*qdotdot*dt^2;
    q0=q
    qdot0=qdot;
    qq(:,i)=q';
end

% [t,x] = ode45(@(t,x)dynamics(x), tspan, x0)
[u,x_des]= control(x0);

for i=1:2
    subplot(2,1,i)
    hold on
    plot(tspan, x_des(i)*ones(size(tspan)), 'r--', 'linewidth', 2);
%     plot(t, x(:,i), 'b', 'linewidth', 2);
    plot(tspan, qq(i,:), 'r', 'linewidth', 0.5);
    hold off
end
% 

clf;
l1=1;
l2=1;
for k=1:10:length(tspan)
    drawcartpend_bw(qq(:,k),l1, l2);
end
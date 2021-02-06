clear; clc; close all;

% x_des=[pi/2; pi; pi/2; pi/2; 0 ; 0; 0 ; 0];

q0= [pi-pi/20; pi-pi/20; pi/3; pi/6]; 
qdot0=[0; 0;0 ;0];
% x_des=[0; 0; pi; pi; 0 ; 0; 0 ; 0];
x0=[q0;qdot0];
% Time settings
t0 = 0;
tf = 5;
dt = 0.001;
% Integration (simulation)
% Create a time vector for numerical integration
tspan = [t0:dt:tf]; 

set(0, 'DefaultTextInterpreter', 'latex') 
set(0, 'DefaultLegendInterpreter', 'latex')
% 

% [t,x] = ode45(@(t,x)dynamics(x), tspan, x0);
[t,x] = ode45(@(t,x)dynamic(x, control(x)), tspan, x0)
% 
[u,x_des]= control(x);

for i=1:2
    subplot(2,1,i)
    hold on
    grid on
    plot(tspan, x_des(i)*ones(size(tspan)), 'r--', 'linewidth', 2);
    plot(t, x(:,i), 'b', 'linewidth', 2);
%     plot(tspan, qq(:,i), 'r', 'linewidth', 0.5);
%     title('q_i')
    hold off
end

%     drawcartpend_bw(qq(k,:),l1, l2);
% end
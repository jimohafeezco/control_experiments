clear; clc; close all;

Q= eye(8)*1000;
R = eye(2)*0.001;

linear.Q = Q;
linear.R = R;

% x_des=[pi/2; pi; pi/2; pi/2; 0 ; 0; 0 ; 0];

q0= [pi-pi/5; pi-pi/5; pi/3; pi/6]; 
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

% evaluati 
[t,x] = ode45(@(t,x)dynamic(x, control(x, linear)), tspan, x0)
% 
[u, s, x_des]= control(x, linear);

jnot= 0;

[t1,J] = ode45(@(t,J)cost(x,linear), tspan, jnot)
figure;
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

figure;
plot(t1, J)
%     drawcartpend_bw(qq(k,:),l1, l2);
% end
clear; clc; close all;


q0= [pi/3; pi/4; pi/3]; 
qdot0=[0; 0; 0];


% x_des=[0; 0; pi; pi; 0 ; 0; 0 ; 0];
x0=[q0;qdot0];

% Time settings`
t0 = 0;
tf = 10;
dt = 0.001;

% Integration (simulation)
% Create a time vector for numerical integration


tspan = [t0:dt:tf]; 
[t,x] = ode45(@(t,x)dynamics(x, control(x)), tspan, x0)

[u, x_des] = control(x);
for i = 1:3
    error(:,i) = x_des(i) - x(:,i);
end


set(0, 'DefaultTextInterpreter', 'latex') 
set(0, 'DefaultLegendInterpreter', 'latex')

xlabel('Time $t$ [s]');
ylabel('State $x$');
subplot(2,1,1)
hold on
plot(t, (x(:,1)), 'r', 'linewidth', 1);
hold on;
grid on;
plot(t, x_des(1)*ones(size(tspan)), 'r--', 'linewidth', 1);
plot(t, (x(:,2)), 'g', 'linewidth', 1);
plot(t, x_des(2)*ones(size(tspan)), 'g--', 'linewidth', 1);
plot(t, (x(:,3)), 'b', 'linewidth', 1);
plot(t, x_des(3)*ones(size(tspan)), 'b--', 'linewidth', 1);
% plot(t, (x(:,3)), 'b', 'linewidth', 1);

legend("x1", 'x2','x3')
title("State variations with time")
hold off

subplot(2,1,2)


plot(t, (error(:,1)), 'r','linewidth', 1);
hold on
grid on

plot(t, (error(:,2)),'g' ,'linewidth', 1);
plot(t, (error(:,3)), 'b','linewidth', 1);
title("Error variations with time")


hold off;

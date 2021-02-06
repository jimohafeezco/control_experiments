% This is main file for simulation, post processing and plotting
clc; % Clears all the text from the Command Window.
clear all; close all; % Remove items from workspace, freeing up system memory
% Set system parameters
m=1; M =5; l = 2; g= -10; 
lambda =0.5;
xdes1 = 1;
xdes2= pi-pi/10;
% Combine parameters to a struct 'system'
system.m = m; 
system.M = M; 
system.l = l; 
system.g = g; 
% Controller parameters
omega = 6;
k = [omega^2 omega^2];
% kd = diag([omega*2 omega*2]);
controller = system;
% controller.k = k;
% controller.kd = kd;
controller.xdes1 = xdes1;
controller.lambda =lambda;
controller.xdes2 = xdes2;
% Trajectory params
A = [1 0.8];
A0 = [2 5];
nu = [0.5 1.5];
trajectory.A = A;
trajectory.A0 = A0;
trajectory.nu = nu;

% Time settings
t0 = 0;
tf = 10;
dt = 0.01;
% Integration (simulation)
% Create a time vector for numerical integration
tspan = [t0:dt:tf]; 
% Set a vector of initial conditions x0 = [theta_0 dthetadt_0]
x0 = [3; pi; 0; 0];
% Run a solver to integrate our differential equations
[t,x] = ode45(@(t,x)dynamics(t, x, system, trajectory,  controller), tspan, x0);
% t - vector of time 
% x - solution of ODE (value of evalueted integral in time span t)

% Calculate control and desired trajectory for solution
[u, x_des] = control(t, x, trajectory, controller);

% Calculate error 
for i = 1:2
    error(:,i) = x_des(:,i) - x(:,i);
end

% Ploting
% Set latex interpreter for plots text and legends
set(0, 'DefaultTextInterpreter', 'latex') 
set(0, 'DefaultLegendInterpreter', 'latex')
% Subplot for time evalution of system state
clf;
subplot(2,1,1);
hold on;

plot(t, x(:,1), 'r', 'linewidth', 0.5);
plot(t, x_des(:,1)*ones(size(t)), 'r--', 'linewidth', 2);
legend('$q_1$', '$q_1des$')

% plot(t, error(:,i), 'g', 'linewidth', 2);
subplot(2,1,2);
hold on
plot(t, x(:,2), 'b', 'linewidth', 0.5);
plot(t, x_des(:,2)*ones(size(t)), 'b--', 'linewidth', 2);
legend('$q_2$',  '$q_2des$')
hold off;
grid on;
% ylim([min([theta; error; theta_des])- 0.2, max([theta; error; theta_des]) + 0.2])
xlabel('Time $t$ [s]');
ylabel('State $x$');
% 
% subplot(3,1,3);
% hold on
% plot(t, error(:,1), 'r', 'linewidth', 2);
% plot(t, error(:,2), 'b', 'linewidth', 2);
% legend('$q_1$', '$q_2$')
% title("Error")
% grid on
% hold off
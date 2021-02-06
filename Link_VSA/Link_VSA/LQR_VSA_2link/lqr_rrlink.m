clear all, close all, clc

m1=1;

m2=1;
l1=1;
l2=2;
g=10;
I1=0.083;
I2=0.33;
x_des=[pi/2; 0; 0; 0];

controller.m1=m1;
controller.m2=m1;
controller.l1=l1;
controller.l2=l2;
controller.g=g;
controller.I1=I1;
controller.I2=I2;
tspan = 0:.01:10;
controller.x_des= x_des;

y0 = [pi; 0 ; 0; 0];

% [t,y] = ode45(@(t,y)((A-B*K)*(y-[0; 0; 0; 0])),tspan,y0);

[t,y] = ode45(@(t,y)dynamics(t,y,controller),tspan,y0);



% Ploting
% Set latex interpreter for plots text and legends
set(0, 'DefaultTextInterpreter', 'latex') 
set(0, 'DefaultLegendInterpreter', 'latex')
% Subplot for time evalution of system state
clf;
subplot(2,1,1);
hold on;
grid on
plot(t, y(:,1), 'r', 'linewidth', 0.5);
plot(t, x_des(1)*ones(size(t)), 'r--', 'linewidth', 2);
legend('$q_1$', '$q_1des$')

% plot(t, error(:,i), 'g', 'linewidth', 2);
subplot(2,1,2);
hold on
plot(t, y(:,2), 'b', 'linewidth', 0.5);
plot(t, x_des(2)*ones(size(t)), 'b--', 'linewidth', 2);
legend('$q_2$',  '$q_2des$')
hold off;
grid on;
% ylim([min([theta; error; theta_des])- 0.2, max([theta; error; theta_des]) + 0.2])
xlabel('Time $t$ [s]');
ylabel('State $x$');
% 
% for k=1:100:length(t)
%     drawcartpend_bw(y(k,:),m,M,l);
% end

% function dy = pendcart(y,m,M,L,g,d,u)
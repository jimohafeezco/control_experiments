clear all, close all, clc

m = 1;
M = 5;
l = 2;
g = -10;
d = 1;
s = 1; 


% pendulum up (s=1)
% A = [0 1 0 0;
%     0 -d/M -m*g/M 0;
%     0 0 0 1;
%     0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0]

dgdx =[0  0;
        0  -g];    

H = [1;0];

D = [M+m   -m*l;
    -1       l];

d1=-inv(D)*dgdx;

A= [ zeros(2,2) eye(2);
    -d1 zeros(2,2)]

B= [0;0; inv(D)* H];

% 
% B = [0; 1/M; 0; s*1/(M*L)];
% eig(A)

Q = [10 0 0 0;
    0 10 0 0;
    0 0 10 0;
    0 0 0 10];

R = 1;

%%
det(ctrb(A,B));
%%
K = lqr(A,B,Q,R);

tspan = 0:.001:10;

y0 = [0; pi-pi/3; 0; 0];
x_des=[3; pi; 0; 0];
% [t,y] = ode45(@(t,y)((A-B*K)*(y-[0; 0; 0; 0])),tspan,y0);

[t,y] = ode45(@(t,y)cartpend(y,m,M,l,g,d,-K*(y-x_des)),tspan,y0);

%   -[3; 0; pi; 0]
% if(s==-1)
%     y0 = [0; 0; pi/3; 0];
%     [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[4; 0; 0; 0])),tspan,y0);
% elseif(s==1)
%     y0 = [-3; 0; pi-0.2; 0];
% % % [t,y] = ode45(@(t,y)((A-B*K)*(y-[0; 0; pi; 0])),tspan,y0);
%     [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[3; 0; pi; 0])),tspan,y0);
%     
% end

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
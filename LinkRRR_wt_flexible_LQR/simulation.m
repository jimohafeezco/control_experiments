clear; clc; close all;

% x_des=[pi/2; pi; pi/2; pi/2; 0 ; 0; 0 ; 0];

q0= [pi-pi/10; pi-pi/10; pi-pi/10; pi /3; pi/6; pi/4]; 
qdot0=[0; 0;0 ;0; 0 ;0];
% x_des=[0; 0; pi; pi; 0 ; 0; 0 ; 0];
x0=[q0;qdot0];
% Time settings
t0 = 0;
tf = 5;
dt = 0.01;
% Integration (simulation)
% Create a time vector for numerical integration
tspan = [t0:dt:tf]; 

set(0, 'DefaultTextInterpreter', 'latex') 
set(0, 'DefaultLegendInterpreter', 'latex')
% 

% [t,x] = ode45(@(t,x)dynamics(x), tspan, x0);
[t,x] = ode45(@(t,x)dynamics(x, control(x)), tspan, x0);
% 
[u,x_des]= control(x);
% 
for i=1:3
    subplot(3,1,i)
    hold on
    plot(tspan, (x_des(i))*ones(size(tspan)), 'r--', 'linewidth', 2);
    plot(t, (x(:,i)), 'b', 'linewidth', 2);
    grid on
    title("Transient response")
    ylabel("radians")
%     plot(tspan, qq(:,i), 'r', 'linewidth', 0.5);
%     title('q_i')
end
% hold off











for i = 1:3
    error(:,i) = -x_des(i) + x(:,i);
end


figure()
subplot(3,1,1)

plot(t, (error(:,1)), 'r','linewidth', 1);
grid on
title("Error q1")
subplot(3,1,2)

plot(t, (error(:,2)),'g' ,'linewidth', 1);
grid on
title("Error q2")

subplot(3,1,3)
plot(t, (error(:,3)), 'b','linewidth', 1);
grid on
title("Error q3")

hold off;


%     drawcartpend_bw(qq(k,:),l1, l2);
% end
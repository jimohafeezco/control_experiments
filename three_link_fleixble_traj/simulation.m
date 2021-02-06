clf;clc; clear; close all;
%SIMULATION Summary of this function goes here
%   Detailed explanation goes here

t0=0;
tf=15;
dt=0.005;
tspan = [t0:dt:tf]; 

% r= task(tspan);
% q= IK(tspan);
% q1= IK(tspan+0.01);
% v= (q1-q)/dt;
% [q, v,r] = IK2(tspan);
% % % % 

traj=0;
if traj
    [q,v] = IK(tspan);
else
    des = [pi/2; pi/2;pi/2;0; 0;0].* ones(6,length(tspan));
    q= des(1:3,:);
    v= des(4:6,:);
end
    
x= zeros(12,length(tspan));
x_ideal= zeros(3, length(tspan));
x_con= zeros(3, length(tspan));

traj= [q(:,:);q(:,:);v(:,:); v(:,:)];
% x(:,1)= [0; 0; 0; 0 ;0; 0; 0; 0];

x(:,1) = [q(:,1)-pi;q(:,1)-pi;v(:,1);v(:,1)];

% x(:,1)= [pi/2;pi/2;pi/2;pi/2;0; 0; 0; 0];

for i=1:length(tspan)-1

x_des = traj(:,i);
    
u  = control(x(:,i),  x_des);

k1 = dynamics(x(:,i), u);
k2 = dynamics(x(:,i)+ dt * k1/2., u);
k3 = dynamics(x(:,i)+ dt * k2/2., u);
k4 = dynamics(x(:,i)+ dt*k3, u);

x(:,i+1) = x(:,i) + dt/6. * (k1 + 2.*k2 + 2.*k3 + k4);

x_con(:,i)   = g_rO3([x(1,i); x(2,i);x(3,i)]);
x_ideal(:,i) = g_rO3([q(1,i); q(2,i);q(3,i)]);
end

for i= 1: length(x)
    drawcartpend_bw(x(:,i),1,1)
end
figure('Color', 'w');
plot(tspan, (x(1,:)), 'r', 'linewidth', 1);
hold on
plot(tspan, (x(2,:)), 'g', 'linewidth', 1);
plot(tspan, (x(3,:)), 'b', 'linewidth', 1);

plot(tspan, (q(1,:)), 'r--', 'linewidth', 1);
plot(tspan, (q(2,:)), 'g--', 'linewidth', 1);
plot(tspan, (q(3,:)), 'b--', 'linewidth', 1);
grid on; 
% grid minor;
ax = gca;
ax.FontName = 'Times New Roman';
% ax.FontSize = 18;
xlabel_handle = xlabel('$$t$$(s)');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('q(rad)');
legend_handle = legend('$$q_1$$', ...
                       '$$q_2$$', ...
                       '$$q_3$$');
legend_handle.Interpreter = 'latex';
xlabel_handle.Interpreter = 'latex';
title("joint positions")

% 
figure('Color', 'w');
% subplot(2,1,2); 
plot(tspan, (q(1,:)-x(1,:)), 'r', 'linewidth', 1);
hold on
plot(tspan, (q(2,:)-x(2,:)), 'g', 'linewidth', 1);
plot(tspan, (q(3,:)-x(3,:)), 'b', 'linewidth', 1);
grid on; 
% grid minor;
ylabel_handle = ylabel('q(rad)');
xlabel_handle = xlabel('$$t$$(s)');
legend_handle.Interpreter = 'latex';
xlabel_handle.Interpreter = 'latex';
title("Error in joint positions")



function u = control(state, x_des)
        
    s= state;
    A = g_Linear_A(s);
    Q = 10000*diag([2; 2; 2; 2;2;2; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5]);

    R = eye(3)*0.1;

    B = g_Linear_B(s);
    K = lqr(A, B, Q, R);

    rank(ctrb(A,B));
    C = g_Linear_C(s);

    u0 = pinv(B) * (-A*s - C);

    e=s-x_des;
    u= -K*e+u0;
end

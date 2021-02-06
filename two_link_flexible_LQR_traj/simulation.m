clf;clc; clear; close all;

%SIMULATION Summary of this function goes here
%   Detailed explanation goes here

t0=0;
tf=15;
dt=0.01;

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
    des = [pi/2; pi/2; 0;0].* ones(4,length(tspan));
    q= des(1:2,:);
    v= des(3:4,:);
end
x= zeros(8,length(tspan));
q_ideal= zeros(3, length(tspan));
traj= [q(:,:);q(:,:);v(:,:); v(:,:)];
% x(:,1)= [0; 0; 0; 0 ;0; 0; 0; 0];

x(:,1) = [q(:,1)-0.7;q(:,1)-0.7;v(:,1);v(:,1)];

% x(:,1)= [pi/2;pi/2;pi/2;pi/2;0; 0; 0; 0];

for i=1:length(tspan)-1

x_des = traj(:,i);
    
u  = control(x(:,i),  x_des);

k1 = dynamics(x(:,i), u);
k2 = dynamics(x(:,i)+ dt * k1/2., u);
k3 = dynamics(x(:,i)+ dt * k2/2., u);
k4 = dynamics(x(:,i)+ dt*k3, u);

x(:,i+1) = x(:,i) + dt/6. * (k1 + 2.*k2 + 2.*k3 + k4);

q_ideal(:,i) = g_rO2([x(1,i); x(2,i)]);

end

figure('Color', 'w');

plot(tspan, (x(1,:)), 'r', 'linewidth', 1.0);
hold on
plot(tspan, (x(2,:)), 'b', 'linewidth', 1.0);
plot(tspan, (q(1,:)), 'r--', 'linewidth', 1);
plot(tspan, (q(2,:)), 'b--', 'linewidth', 1);
grid on; 
ax = gca;
ax.FontName = 'Times New Roman';
% ax.FontSize = 18;
xlabel_handle = xlabel('$$t$$(s)');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('q(rad)');
legend_handle = legend('$$q_1$$', ...
                       '$$q_2$$');
legend_handle.Interpreter = 'latex';
xlabel_handle.Interpreter = 'latex';
title("joint positions")

% 
% figure('Color', 'w');
% plot(q_ideal(1,:),q_ideal(2,:))
% hold on
% plot(r(1,:),r(2,:),'y')


figure('Color', 'w');
% subplot(2,1,2); 
plot(tspan, (q(1,:)-x(1,:)), 'r', 'linewidth', 1.0);
hold on
plot(tspan, (q(2,:)-x(2,:)), 'g', 'linewidth', 1.0);
grid on; 
ylabel_handle = ylabel('q(rad)');
xlabel_handle = xlabel('$$t$$(s)');
legend_handle.Interpreter = 'latex';
xlabel_handle.Interpreter = 'latex';
title("Error in joint positions")


function u = control(state, x_des)
        
    s= state;
    A = g_Linear_A(s);
    Q = 10000*diag([0.1; 0.1; 0.1; 0.1; 0.01; 0.01; 0.01; 0.01]);
    R = eye(2)*0.1;

    B = g_Linear_B(s);
    G = g_constraint(s);
    N= null(G);
    
    An = N'*A*N;
    Bn = N'*B;
    Qn = N'*Q*N;
    
    Kn = lqr(An, Bn, Qn, R);
    K = Kn*N'

    C = g_Linear_C(s);

    u0 = pinv(B) * (-A*s - C);

    e=s-x_des;
    u= -K*e+u0;
end

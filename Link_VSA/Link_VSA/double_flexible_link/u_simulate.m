clc; clear; close all;

dof = 2;

Time = 5;
dt = 0.005;
Count = floor(Time/ dt);

Res.Time = zeros(Count, 1);
Res.Position = zeros(Count, dof);
Res.Position_Theta = zeros(Count, dof);
Res.Position_Desired = zeros(Count, dof);
Res.Position_rO2 = zeros(Count, 2);
Res.Position_rO2_desired = zeros(Count, 2);
Res.Spring = zeros(Count, dof);
Res.Spring_Desired = zeros(Count, dof);


x = ControlInput_x(0, dt);

for i = 1:Count
    
    t = i*dt;
    
%     [x_desired, dx_desired] = ControlInput_x(t, dt);
    [x_desired, dx_desired] = ControlInput_x_PID(t, dt, x);
    
    u0 = ID_noacc(x_desired, dx_desired);%ID_noacc ID
    u = LQR_controller(x, x_desired);
    
    x = RungeUpdate(x, u+u0, dt);
    
    
    Res.Time(i, :) = t;
    
    Res.Position(i, :) = x(5:6);
    Res.Position_Theta(i, :) = x(7:8);
    Res.Position_Desired(i, :) = x_desired(5:6);
    Res.Spring(i, :) = x(9:10);
    Res.Spring_Desired(i, :) = x_desired(9:10);
    
    rO2 = g_rO2(x(5:6));
    Res.Position_rO2(i, :) = rO2(1:2);
    Res.Position_rO2_desired(i, :) = task(t);
end
    
figure('Color', 'w');
plot(Res.Time, Res.Position, 'LineWidth', 2); hold on;
plot(Res.Time, Res.Position_Theta, 'LineWidth', 2, 'LineStyle', ':');
plot(Res.Time, Res.Position_Desired, 'LineWidth', 3, 'LineStyle', '--');
grid on; grid minor;
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 18;
xlabel_handle = xlabel('$$t$$, s');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$\varphi$$ (rad)');
ylabel_handle.Interpreter = 'latex';
legend_handle = legend('$$\varphi_1$$ (rad)', '$$\varphi_2$$ (rad)', ...
                       '$$\theta_1$$ (rad)', '$$\theta_2$$ (rad)', ...
                       '$$\varphi_1^*$$ (rad)', '$$\varphi_2^*$$ (rad)');
legend_handle.Interpreter = 'latex';


figure('Color', 'w');
plot(Res.Time, Res.Position(:, 1), 'LineWidth', 2); hold on;
plot(Res.Time, Res.Position_Theta(:, 1), 'LineWidth', 2, 'LineStyle', ':');
plot(Res.Time, Res.Position_Desired(:, 1), 'LineWidth', 3, 'LineStyle', '--');
grid on; grid minor;
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 18;
xlabel_handle = xlabel('$$t$$, s');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$q_1$$, $$\theta_1$$, $$q_1^*$$ (rad)');
ylabel_handle.Interpreter = 'latex';
legend_handle = legend('$$q_1$$ (rad)', ...
                       '$$\theta_1$$ (rad)', ...
                       '$$q_1^*$$ (rad)');
legend_handle.Interpreter = 'latex';


figure('Color', 'w');
plot(Res.Position_rO2(:, 1), Res.Position_rO2(:, 2), 'LineWidth', 1); hold on;
plot(Res.Position_rO2_desired(:, 1), Res.Position_rO2_desired(:, 2), 'LineWidth', 2, 'LineStyle', '--'); grid on; grid minor;
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 18;
xlabel_handle = xlabel('$$x$$, m');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$y$$, m');
ylabel_handle.Interpreter = 'latex';
legend_handle = legend('$$r_K$$', '$$r_K^*$$');
legend_handle.Interpreter = 'latex';

% figure('Color', 'w');
% plot(Res.Time, Res.Spring, 'LineWidth', 2); hold on;
% plot(Res.Time, Res.Spring_Desired, 'LineWidth', 3, 'LineStyle', '--');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function u0 = ID(x, dx)

A = g_Linear_A(x);
B = g_Linear_B(x);
C = g_Linear_C(x);

u0 = pinv(B) * (dx - A*x - C);

end
function u0 = ID_noacc(x, ~)

A = g_Linear_A(x);
B = g_Linear_B(x);
C = g_Linear_C(x);

u0 = pinv(B) * (-A*x - C);
end

function u = LQR_controller(x, x_desired)

Q = 1000*diag([0.05; 0.05; 0.01; 0.01; 1; 1; 0; 0; 0.01; 0.01]);
R = eye(4) * 0.001;

A = g_Linear_A(x);
B = g_Linear_B(x);

K = lqr(A, B, Q, R);

u = K * (x_desired - x);

end

function u = PD(x, q_desired, v_desired, k_desired)

Kp = eye(2) * 100;
Kd = eye(2) * 50;

Kp_spring = eye(2) * 10;

q = x(5:6);
v = x(7:8);
k = x(9:10);

u = [Kp*(q_desired - q) + Kd*(v_desired - v);
     Kp_spring * (k_desired - k)];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = IK(r)

L = [0.5; 0.5];

h = norm(r);
alpha = atan2(r(2), r(1));

q1 = alpha - acos((L(1)^2 + h^2 - L(2)^2) / (2*L(1)*h)); 
q2 = alpha + acos((L(2)^2 + h^2 - L(1)^2) / (2*L(2)*h)) - q1;  

q = [q1; q2];
end

function r = task(t)

phi = 2*t;
R = 0.05; Center = [0.4; 0.4];

r = Center + [R*cos(phi); R*sin(phi)] + 0.2*[R*cos(5*phi); R*sin(5*phi)];
% r = Center + [R*cos(phi); R*sin(phi)];

end

function [q, v, a, k, dk] = ControlInput(t, dt)
    r0 = task(t);
    q0 = IK(r0);
    
    r1 = task(t + dt);
    q1 = IK(r1);
    
    r2 = task(t + dt);
    q2 = IK(r2);
    
    q = q0;
    v = (q1 - q0) / dt;
    a = (q0 - 2*q1 + q2) / dt^2;
    
    Time = 2;
    
    dk = [700; 700]/Time;
    k = [1000; 1000] + t*dk;
end


function [x, dx] = ControlInput_x(t, dt)

[q, v, a, k, dk] = ControlInput(t, dt);

%K*(q_theta - q) - c = 0
%q_theta - q = K \ c
c = g_c(q, v);
K = diag(k);
theta = K \ c + q;

x = [v; v; q; theta; k];
dx = [a; a; v; v; dk];

end


function [x_desired, dx_desired] = ControlInput_x_PID(t, dt, x)

[q_desired, v_desired, a_desired, k_desired, dk_desired] = ControlInput(t, dt);

c = g_c(q_desired, v_desired); 
H = g_H(q_desired);

e = q_desired - x(5:6);
de = v_desired - x(1:2);
Kp = eye(2)*600;
Kd = eye(2)*20;

u0 = H*(Kd*de + Kp*e) + c;
K = diag(k_desired);
theta = K \ u0 + q_desired;


x_desired = [v_desired; v_desired; q_desired; theta; k_desired];
dx_desired = [a_desired; a_desired; v_desired; v_desired; dk_desired];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function x = RungeUpdate(x, u, dt)
    
    k1 = g_LHS(x,             u);
    k2 = g_LHS(x + k1*0.5*dt, u);
    k3 = g_LHS(x + k2*0.5*dt, u);
    k4 = g_LHS(x + k3*dt,     u);
    
    dx = (k1 + 2*k2 + 2*k3 + k4) / 6;
    
    x = x + dx*dt;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


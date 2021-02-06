function [M,RHS] = eqOfmotion(theta_pos,theta_vel,torque)
% Computes the numerical Equation of Motion of a Five Link Robot using Lagragian Dynamics based on 
% Lecture Notes from Introduction to Robotics, H. Harry Asada - Chapter 7. Page 8 to 16.
% Last modification: 30/03/2014 (by Feliphe G. Galiza)

% parameters according to human body
height = 1.5; % [m]
L1 = 0.285*height; % [m]
L2 = (0.53 - 0.285)*height; % [m]
L3 = height - (L1 + L2); % [m]
L4 = L2; % [m]
L5 =L1; % [m]

r1 = L1/2; % [m]
r2 = L2/2; % [m]
r3 = L3/2; % [m]    
r4 = L4/2; % [m]
r5 = L5/2; % [m]

mass = 50; % [kg]
m1 = 0.061 * mass; % [kg]
m2 = 0.1 * mass; % [kg]
m3 = 0.678 * mass; % [kg]
m4 = m2; % [kg]
m5 = m1; % [kg]

Izz_1 = m1 * 0.735^2; % [kg*m^2]
Izz_2 = m2 * 0.540^2; % [kg*m^2]
Izz_3 = m3 * 0.0798^2; % [kg*m^2]
Izz_4 = Izz_2; % [kg*m^2]
Izz_5 = Izz_1; % [kg*m^2]

% Izz_oint angles
theta1 = theta_pos(1);
theta2 = theta_pos(2);
theta3= theta_pos(3);
theta4 = theta_pos(4);
theta5 = theta_pos(5);

% Izz_oint velocities
theta1_dot = theta_vel(1);
theta2_dot = theta_vel(2);
theta3_dot = theta_vel(3);
theta4_dot  = theta_vel(4);
theta5_dot = theta_vel(5);

% Izz_oint torques
u1 = torque(1);
u2 = torque(2);
u3 = torque(3);
u4 = torque(4);
u5 = torque(5);

g = 9.81; % [m/s^2]
% equation of motion

% Using Newton-Euler
M(1,:) = [ Izz_1 + Izz_2 + Izz_3 + Izz_4 + Izz_5 + m5*(r5*sin(theta1 + theta2 + theta3 - theta4 + theta5) - sin(theta1 + theta2)*(L2 - r2) - sin(theta1)*(L1 - r1) + r4*sin(theta1 + theta2 + theta3 - theta4))^2 + m4*(cos(theta1 + theta2)*(L2 - r2) + cos(theta1)*(L1 - r1) - r4*cos(theta1 + theta2 + theta3 - theta4))^2 + m3*(cos(theta1 + theta2)*(L2 - r2) + cos(theta1)*(L1 - r1) + r3*cos(theta1 + theta2 + theta3))^2 + m4*(sin(theta1 + theta2)*(L2 - r2) + sin(theta1)*(L1 - r1) - r4*sin(theta1 + theta2 + theta3 - theta4))^2 + m5*(r5*cos(theta1 + theta2 + theta3 - theta4 + theta5) - cos(theta1 + theta2)*(L2 - r2) - cos(theta1)*(L1 - r1) + r4*cos(theta1 + theta2 + theta3 - theta4))^2 + m2*(cos(theta1 + theta2)*(L2 - r2) + cos(theta1)*(L1 - r1))^2 + m2*(sin(theta1 + theta2)*(L2 - r2) + sin(theta1)*(L1 - r1))^2 + m3*(r3*sin(theta1 + theta2 + theta3) + sin(theta1 + theta2)*(L2 - r2) + sin(theta1)*(L1 - r1))^2 + m1*cos(theta1)^2*(L1 - r1)^2 + m1*sin(theta1)^2*(L1 - r1)^2, Izz_2 + Izz_3 + Izz_4 + Izz_5 + L2^2*m2 + L2^2*m3 + L2^2*m4 + L2^2*m5 + m2*r2^2 + m3*r2^2 + m3*r3^2 + m4*r2^2 + m5*r2^2 + m4*r4^2 + m5*r4^2 + m5*r5^2 - 2*L2*m2*r2 - 2*L2*m3*r2 - 2*L2*m4*r2 - 2*L2*m5*r2 - L1*m4*r4*cos(theta2 + theta3 - theta4) - L1*m5*r4*cos(theta2 + theta3 - theta4) - 2*L2*m5*r5*cos(theta3 - theta4 + theta5) + m4*r1*r4*cos(theta2 + theta3 - theta4) + m5*r1*r4*cos(theta2 + theta3 - theta4) + 2*m5*r2*r5*cos(theta3 - theta4 + theta5) + L1*m3*r3*cos(theta2 + theta3) + L1*L2*m2*cos(theta2) + L1*L2*m3*cos(theta2) + L1*L2*m4*cos(theta2) + L1*L2*m5*cos(theta2) - m3*r1*r3*cos(theta2 + theta3) - L1*m2*r2*cos(theta2) - L2*m2*r1*cos(theta2) - L1*m3*r2*cos(theta2) - L2*m3*r1*cos(theta2) - L1*m4*r2*cos(theta2) - L2*m4*r1*cos(theta2) - L1*m5*r2*cos(theta2) - L2*m5*r1*cos(theta2) + 2*L2*m3*r3*cos(theta3) - L1*m5*r5*cos(theta2 + theta3 - theta4 + theta5) + m2*r1*r2*cos(theta2) + m3*r1*r2*cos(theta2) + m4*r1*r2*cos(theta2) + m5*r1*r2*cos(theta2) - 2*m3*r2*r3*cos(theta3) + 2*m5*r4*r5*cos(theta5) - 2*L2*m4*r4*cos(theta3 - theta4) - 2*L2*m5*r4*cos(theta3 - theta4) + m5*r1*r5*cos(theta2 + theta3 - theta4 + theta5) + 2*m4*r2*r4*cos(theta3 - theta4) + 2*m5*r2*r4*cos(theta3 - theta4), Izz_3 + Izz_4 + Izz_5 + m3*r3^2 + m4*r4^2 + m5*r4^2 + m5*r5^2 - L1*m4*r4*cos(theta2 + theta3 - theta4) - L1*m5*r4*cos(theta2 + theta3 - theta4) - L2*m5*r5*cos(theta3 - theta4 + theta5) + m4*r1*r4*cos(theta2 + theta3 - theta4) + m5*r1*r4*cos(theta2 + theta3 - theta4) + m5*r2*r5*cos(theta3 - theta4 + theta5) + L1*m3*r3*cos(theta2 + theta3) - m3*r1*r3*cos(theta2 + theta3) + L2*m3*r3*cos(theta3) - L1*m5*r5*cos(theta2 + theta3 - theta4 + theta5) - m3*r2*r3*cos(theta3) + 2*m5*r4*r5*cos(theta5) - L2*m4*r4*cos(theta3 - theta4) - L2*m5*r4*cos(theta3 - theta4) + m5*r1*r5*cos(theta2 + theta3 - theta4 + theta5) + m4*r2*r4*cos(theta3 - theta4) + m5*r2*r4*cos(theta3 - theta4), L1*m4*r4*cos(theta2 + theta3 - theta4) - Izz_5 - m4*r4^2 - m5*r4^2 - m5*r5^2 - Izz_4 + L1*m5*r4*cos(theta2 + theta3 - theta4) + L2*m5*r5*cos(theta3 - theta4 + theta5) - m4*r1*r4*cos(theta2 + theta3 - theta4) - m5*r1*r4*cos(theta2 + theta3 - theta4) - m5*r2*r5*cos(theta3 - theta4 + theta5) + L1*m5*r5*cos(theta2 + theta3 - theta4 + theta5) - 2*m5*r4*r5*cos(theta5) + L2*m4*r4*cos(theta3 - theta4) + L2*m5*r4*cos(theta3 - theta4) - m5*r1*r5*cos(theta2 + theta3 - theta4 + theta5) - m4*r2*r4*cos(theta3 - theta4) - m5*r2*r4*cos(theta3 - theta4), Izz_5 + m5*r5^2 - L2*m5*r5*cos(theta3 - theta4 + theta5) + m5*r2*r5*cos(theta3 - theta4 + theta5) - L1*m5*r5*cos(theta2 + theta3 - theta4 + theta5) + m5*r4*r5*cos(theta5) + m5*r1*r5*cos(theta2 + theta3 - theta4 + theta5)];
M(2,:) = [ Izz_2 + Izz_3 + Izz_4 + Izz_5 + L2^2*m2 + L2^2*m3 + L2^2*m4 + L2^2*m5 + m2*r2^2 + m3*r2^2 + m3*r3^2 + m4*r2^2 + m5*r2^2 + m4*r4^2 + m5*r4^2 + m5*r5^2 - 2*L2*m2*r2 - 2*L2*m3*r2 - 2*L2*m4*r2 - 2*L2*m5*r2 - L1*m4*r4*cos(theta2 + theta3 - theta4) - L1*m5*r4*cos(theta2 + theta3 - theta4) - 2*L2*m5*r5*cos(theta3 - theta4 + theta5) + m4*r1*r4*cos(theta2 + theta3 - theta4) + m5*r1*r4*cos(theta2 + theta3 - theta4) + 2*m5*r2*r5*cos(theta3 - theta4 + theta5) + L1*m3*r3*cos(theta2 + theta3) + L1*L2*m2*cos(theta2) + L1*L2*m3*cos(theta2) + L1*L2*m4*cos(theta2) + L1*L2*m5*cos(theta2) - m3*r1*r3*cos(theta2 + theta3) - L1*m2*r2*cos(theta2) - L2*m2*r1*cos(theta2) - L1*m3*r2*cos(theta2) - L2*m3*r1*cos(theta2) - L1*m4*r2*cos(theta2) - L2*m4*r1*cos(theta2) - L1*m5*r2*cos(theta2) - L2*m5*r1*cos(theta2) + 2*L2*m3*r3*cos(theta3) - L1*m5*r5*cos(theta2 + theta3 - theta4 + theta5) + m2*r1*r2*cos(theta2) + m3*r1*r2*cos(theta2) + m4*r1*r2*cos(theta2) + m5*r1*r2*cos(theta2) - 2*m3*r2*r3*cos(theta3) + 2*m5*r4*r5*cos(theta5) - 2*L2*m4*r4*cos(theta3 - theta4) - 2*L2*m5*r4*cos(theta3 - theta4) + m5*r1*r5*cos(theta2 + theta3 - theta4 + theta5) + 2*m4*r2*r4*cos(theta3 - theta4) + 2*m5*r2*r4*cos(theta3 - theta4), Izz_2 + Izz_3 + Izz_4 + Izz_5 + L2^2*m2 + L2^2*m3 + L2^2*m4 + L2^2*m5 + m2*r2^2 + m3*r2^2 + m3*r3^2 + m4*r2^2 + m5*r2^2 + m4*r4^2 + m5*r4^2 + m5*r5^2 - 2*L2*m2*r2 - 2*L2*m3*r2 - 2*L2*m4*r2 - 2*L2*m5*r2 - 2*L2*m5*r5*cos(theta3 - theta4 + theta5) + 2*m5*r2*r5*cos(theta3 - theta4 + theta5) + 2*L2*m3*r3*cos(theta3) - 2*m3*r2*r3*cos(theta3) + 2*m5*r4*r5*cos(theta5) - 2*L2*m4*r4*cos(theta3 - theta4) - 2*L2*m5*r4*cos(theta3 - theta4) + 2*m4*r2*r4*cos(theta3 - theta4) + 2*m5*r2*r4*cos(theta3 - theta4), Izz_3 + Izz_4 + Izz_5 + m3*r3^2 + m4*r4^2 + m5*r4^2 + m5*r5^2 - L2*m5*r5*cos(theta3 - theta4 + theta5) + m5*r2*r5*cos(theta3 - theta4 + theta5) + L2*m3*r3*cos(theta3) - m3*r2*r3*cos(theta3) + 2*m5*r4*r5*cos(theta5) - L2*m4*r4*cos(theta3 - theta4) - L2*m5*r4*cos(theta3 - theta4) + m4*r2*r4*cos(theta3 - theta4) + m5*r2*r4*cos(theta3 - theta4), L2*m5*r5*cos(theta3 - theta4 + theta5) - Izz_5 - m4*r4^2 - m5*r4^2 - m5*r5^2 - Izz_4 - m5*r2*r5*cos(theta3 - theta4 + theta5) - 2*m5*r4*r5*cos(theta5) + L2*m4*r4*cos(theta3 - theta4) + L2*m5*r4*cos(theta3 - theta4) - m4*r2*r4*cos(theta3 - theta4) - m5*r2*r4*cos(theta3 - theta4), Izz_5 + m5*r5^2 - L2*m5*r5*cos(theta3 - theta4 + theta5) + m5*r2*r5*cos(theta3 - theta4 + theta5) + m5*r4*r5*cos(theta5)];
M(3,:) = [ Izz_3 + Izz_4 + Izz_5 + m3*r3^2 + m4*r4^2 + m5*r4^2 + m5*r5^2 - L1*m4*r4*cos(theta2 + theta3 - theta4) - L1*m5*r4*cos(theta2 + theta3 - theta4) - L2*m5*r5*cos(theta3 - theta4 + theta5) + m4*r1*r4*cos(theta2 + theta3 - theta4) + m5*r1*r4*cos(theta2 + theta3 - theta4) + m5*r2*r5*cos(theta3 - theta4 + theta5) + L1*m3*r3*cos(theta2 + theta3) - m3*r1*r3*cos(theta2 + theta3) + L2*m3*r3*cos(theta3) - L1*m5*r5*cos(theta2 + theta3 - theta4 + theta5) - m3*r2*r3*cos(theta3) + 2*m5*r4*r5*cos(theta5) - L2*m4*r4*cos(theta3 - theta4) - L2*m5*r4*cos(theta3 - theta4) + m5*r1*r5*cos(theta2 + theta3 - theta4 + theta5) + m4*r2*r4*cos(theta3 - theta4) + m5*r2*r4*cos(theta3 - theta4), Izz_3 + Izz_4 + Izz_5 + m3*r3^2 + m4*r4^2 + m5*r4^2 + m5*r5^2 - L2*m5*r5*cos(theta3 - theta4 + theta5) + m5*r2*r5*cos(theta3 - theta4 + theta5) + L2*m3*r3*cos(theta3) - m3*r2*r3*cos(theta3) + 2*m5*r4*r5*cos(theta5) - L2*m4*r4*cos(theta3 - theta4) - L2*m5*r4*cos(theta3 - theta4) + m4*r2*r4*cos(theta3 - theta4) + m5*r2*r4*cos(theta3 - theta4), Izz_3 + Izz_4 + Izz_5 + m3*r3^2 + m4*r4^2 + m5*r4^2 + m5*r5^2 + 2*m5*r4*r5*cos(theta5), - Izz_4 - Izz_5 - m4*r4^2 - m5*r4^2 - m5*r5^2 - 2*m5*r4*r5*cos(theta5), m5*r5^2 + m5*r4*cos(theta5)*r5 + Izz_5];
M(4,:) = [ L1*m4*r4*cos(theta2 + theta3 - theta4) - Izz_5 - m4*r4^2 - m5*r4^2 - m5*r5^2 - Izz_4 + L1*m5*r4*cos(theta2 + theta3 - theta4) + L2*m5*r5*cos(theta3 - theta4 + theta5) - m4*r1*r4*cos(theta2 + theta3 - theta4) - m5*r1*r4*cos(theta2 + theta3 - theta4) - m5*r2*r5*cos(theta3 - theta4 + theta5) + L1*m5*r5*cos(theta2 + theta3 - theta4 + theta5) - 2*m5*r4*r5*cos(theta5) + L2*m4*r4*cos(theta3 - theta4) + L2*m5*r4*cos(theta3 - theta4) - m5*r1*r5*cos(theta2 + theta3 - theta4 + theta5) - m4*r2*r4*cos(theta3 - theta4) - m5*r2*r4*cos(theta3 - theta4), L2*m5*r5*cos(theta3 - theta4 + theta5) - Izz_5 - m4*r4^2 - m5*r4^2 - m5*r5^2 - Izz_4 - m5*r2*r5*cos(theta3 - theta4 + theta5) - 2*m5*r4*r5*cos(theta5) + L2*m4*r4*cos(theta3 - theta4) + L2*m5*r4*cos(theta3 - theta4) - m4*r2*r4*cos(theta3 - theta4) - m5*r2*r4*cos(theta3 - theta4), - Izz_4 - Izz_5 - m4*r4^2 - m5*r4^2 - m5*r5^2 - 2*m5*r4*r5*cos(theta5), Izz_4 + Izz_5 + m4*r4^2 + m5*r4^2 + m5*r5^2 + 2*m5*r4*r5*cos(theta5), - m5*r5^2 - m5*r4*cos(theta5)*r5 - Izz_5];
M(5,:) = [ Izz_5 + m5*r5^2 - L2*m5*r5*cos(theta3 - theta4 + theta5) + m5*r2*r5*cos(theta3 - theta4 + theta5) - L1*m5*r5*cos(theta2 + theta3 - theta4 + theta5) + m5*r4*r5*cos(theta5) + m5*r1*r5*cos(theta2 + theta3 - theta4 + theta5), Izz_5 + m5*r5^2 - L2*m5*r5*cos(theta3 - theta4 + theta5) + m5*r2*r5*cos(theta3 - theta4 + theta5) + m5*r4*r5*cos(theta5), m5*r5^2 + m5*r4*cos(theta5)*r5 + Izz_5, - m5*r5^2 - m5*r4*cos(theta5)*r5 - Izz_5, m5*r5^2 + Izz_5];

RHS(1,1) = u1 - g*m5*(r5*sin(theta1 + theta2 + theta3 - theta4 + theta5) - sin(theta1 + theta2)*(L2 - r2) - sin(theta1)*(L1 - r1) + r4*sin(theta1 + theta2 + theta3 - theta4)) + g*m4*(sin(theta1 + theta2)*(L2 - r2) + sin(theta1)*(L1 - r1) - r4*sin(theta1 + theta2 + theta3 - theta4)) + g*m2*(sin(theta1 + theta2)*(L2 - r2) + sin(theta1)*(L1 - r1)) + g*m3*(r3*sin(theta1 + theta2 + theta3) + sin(theta1 + theta2)*(L2 - r2) + sin(theta1)*(L1 - r1)) + g*m1*sin(theta1)*(L1 - r1) + m4*r1*r4*theta2_dot^2*sin(theta2 + theta3 - theta4) + m4*r1*r4*theta3_dot^2*sin(theta2 + theta3 - theta4) + m5*r1*r4*theta2_dot^2*sin(theta2 + theta3 - theta4) + m4*r1*r4*theta4_dot^2*sin(theta2 + theta3 - theta4) + m5*r1*r4*theta3_dot^2*sin(theta2 + theta3 - theta4) + m5*r1*r4*theta4_dot^2*sin(theta2 + theta3 - theta4) + m5*r2*r5*theta3_dot^2*sin(theta3 - theta4 + theta5) + m5*r2*r5*theta4_dot^2*sin(theta3 - theta4 + theta5) + m5*r2*r5*theta5_dot^2*sin(theta3 - theta4 + theta5) + L1*m3*r3*theta2_dot^2*sin(theta2 + theta3) + L1*m3*r3*theta3_dot^2*sin(theta2 + theta3) + L1*L2*m2*theta2_dot^2*sin(theta2) + L1*L2*m3*theta2_dot^2*sin(theta2) + L1*L2*m4*theta2_dot^2*sin(theta2) + L1*L2*m5*theta2_dot^2*sin(theta2) - m3*r1*r3*theta2_dot^2*sin(theta2 + theta3) - m3*r1*r3*theta3_dot^2*sin(theta2 + theta3) - L1*m2*r2*theta2_dot^2*sin(theta2) - L2*m2*r1*theta2_dot^2*sin(theta2) - L1*m3*r2*theta2_dot^2*sin(theta2) - L2*m3*r1*theta2_dot^2*sin(theta2) - L1*m4*r2*theta2_dot^2*sin(theta2) - L2*m4*r1*theta2_dot^2*sin(theta2) - L1*m5*r2*theta2_dot^2*sin(theta2) - L2*m5*r1*theta2_dot^2*sin(theta2) + L2*m3*r3*theta3_dot^2*sin(theta3) - L1*m5*r5*theta2_dot^2*sin(theta2 + theta3 - theta4 + theta5) - L1*m5*r5*theta3_dot^2*sin(theta2 + theta3 - theta4 + theta5) - L1*m5*r5*theta4_dot^2*sin(theta2 + theta3 - theta4 + theta5) - L1*m5*r5*theta5_dot^2*sin(theta2 + theta3 - theta4 + theta5) + m2*r1*r2*theta2_dot^2*sin(theta2) + m3*r1*r2*theta2_dot^2*sin(theta2) + m4*r1*r2*theta2_dot^2*sin(theta2) + m5*r1*r2*theta2_dot^2*sin(theta2) - m3*r2*r3*theta3_dot^2*sin(theta3) + m5*r4*r5*theta5_dot^2*sin(theta5) - L2*m4*r4*theta3_dot^2*sin(theta3 - theta4) - L2*m4*r4*theta4_dot^2*sin(theta3 - theta4) - L2*m5*r4*theta3_dot^2*sin(theta3 - theta4) - L2*m5*r4*theta4_dot^2*sin(theta3 - theta4) + m5*r1*r5*theta2_dot^2*sin(theta2 + theta3 - theta4 + theta5) + m5*r1*r5*theta3_dot^2*sin(theta2 + theta3 - theta4 + theta5) + m5*r1*r5*theta4_dot^2*sin(theta2 + theta3 - theta4 + theta5) + m5*r1*r5*theta5_dot^2*sin(theta2 + theta3 - theta4 + theta5) + m4*r2*r4*theta3_dot^2*sin(theta3 - theta4) + m4*r2*r4*theta4_dot^2*sin(theta3 - theta4) + m5*r2*r4*theta3_dot^2*sin(theta3 - theta4) + m5*r2*r4*theta4_dot^2*sin(theta3 - theta4) - L1*m4*r4*theta2_dot^2*sin(theta2 + theta3 - theta4) - L1*m4*r4*theta3_dot^2*sin(theta2 + theta3 - theta4) - L1*m5*r4*theta2_dot^2*sin(theta2 + theta3 - theta4) - L1*m4*r4*theta4_dot^2*sin(theta2 + theta3 - theta4) - L1*m5*r4*theta3_dot^2*sin(theta2 + theta3 - theta4) - L1*m5*r4*theta4_dot^2*sin(theta2 + theta3 - theta4) - L2*m5*r5*theta3_dot^2*sin(theta3 - theta4 + theta5) - L2*m5*r5*theta4_dot^2*sin(theta3 - theta4 + theta5) - L2*m5*r5*theta5_dot^2*sin(theta3 - theta4 + theta5) - 2*L1*m4*r4*theta1_dot*theta2_dot*sin(theta2 + theta3 - theta4) - 2*L1*m4*r4*theta1_dot*theta3_dot*sin(theta2 + theta3 - theta4) - 2*L1*m5*r4*theta1_dot*theta2_dot*sin(theta2 + theta3 - theta4) + 2*L1*m4*r4*theta1_dot*theta4_dot*sin(theta2 + theta3 - theta4) - 2*L1*m4*r4*theta2_dot*theta3_dot*sin(theta2 + theta3 - theta4) - 2*L1*m5*r4*theta1_dot*theta3_dot*sin(theta2 + theta3 - theta4) + 2*L1*m4*r4*theta2_dot*theta4_dot*sin(theta2 + theta3 - theta4) + 2*L1*m5*r4*theta1_dot*theta4_dot*sin(theta2 + theta3 - theta4) - 2*L1*m5*r4*theta2_dot*theta3_dot*sin(theta2 + theta3 - theta4) + 2*L1*m4*r4*theta3_dot*theta4_dot*sin(theta2 + theta3 - theta4) + 2*L1*m5*r4*theta2_dot*theta4_dot*sin(theta2 + theta3 - theta4) + 2*L1*m5*r4*theta3_dot*theta4_dot*sin(theta2 + theta3 - theta4) - 2*L2*m5*r5*theta1_dot*theta3_dot*sin(theta3 - theta4 + theta5) + 2*L2*m5*r5*theta1_dot*theta4_dot*sin(theta3 - theta4 + theta5) - 2*L2*m5*r5*theta2_dot*theta3_dot*sin(theta3 - theta4 + theta5) - 2*L2*m5*r5*theta1_dot*theta5_dot*sin(theta3 - theta4 + theta5) + 2*L2*m5*r5*theta2_dot*theta4_dot*sin(theta3 - theta4 + theta5) - 2*L2*m5*r5*theta2_dot*theta5_dot*sin(theta3 - theta4 + theta5) + 2*L2*m5*r5*theta3_dot*theta4_dot*sin(theta3 - theta4 + theta5) - 2*L2*m5*r5*theta3_dot*theta5_dot*sin(theta3 - theta4 + theta5) + 2*L2*m5*r5*theta4_dot*theta5_dot*sin(theta3 - theta4 + theta5) + 2*m4*r1*r4*theta1_dot*theta2_dot*sin(theta2 + theta3 - theta4) + 2*m4*r1*r4*theta1_dot*theta3_dot*sin(theta2 + theta3 - theta4) + 2*m5*r1*r4*theta1_dot*theta2_dot*sin(theta2 + theta3 - theta4) - 2*m4*r1*r4*theta1_dot*theta4_dot*sin(theta2 + theta3 - theta4) + 2*m4*r1*r4*theta2_dot*theta3_dot*sin(theta2 + theta3 - theta4) + 2*m5*r1*r4*theta1_dot*theta3_dot*sin(theta2 + theta3 - theta4) - 2*m4*r1*r4*theta2_dot*theta4_dot*sin(theta2 + theta3 - theta4) - 2*m5*r1*r4*theta1_dot*theta4_dot*sin(theta2 + theta3 - theta4) + 2*m5*r1*r4*theta2_dot*theta3_dot*sin(theta2 + theta3 - theta4) - 2*m4*r1*r4*theta3_dot*theta4_dot*sin(theta2 + theta3 - theta4) - 2*m5*r1*r4*theta2_dot*theta4_dot*sin(theta2 + theta3 - theta4) - 2*m5*r1*r4*theta3_dot*theta4_dot*sin(theta2 + theta3 - theta4) + 2*m5*r2*r5*theta1_dot*theta3_dot*sin(theta3 - theta4 + theta5) - 2*m5*r2*r5*theta1_dot*theta4_dot*sin(theta3 - theta4 + theta5) + 2*m5*r2*r5*theta2_dot*theta3_dot*sin(theta3 - theta4 + theta5) + 2*m5*r2*r5*theta1_dot*theta5_dot*sin(theta3 - theta4 + theta5) - 2*m5*r2*r5*theta2_dot*theta4_dot*sin(theta3 - theta4 + theta5) + 2*m5*r2*r5*theta2_dot*theta5_dot*sin(theta3 - theta4 + theta5) - 2*m5*r2*r5*theta3_dot*theta4_dot*sin(theta3 - theta4 + theta5) + 2*m5*r2*r5*theta3_dot*theta5_dot*sin(theta3 - theta4 + theta5) - 2*m5*r2*r5*theta4_dot*theta5_dot*sin(theta3 - theta4 + theta5) + 2*L1*m3*r3*theta1_dot*theta2_dot*sin(theta2 + theta3) + 2*L1*m3*r3*theta1_dot*theta3_dot*sin(theta2 + theta3) + 2*L1*m3*r3*theta2_dot*theta3_dot*sin(theta2 + theta3) + 2*L1*L2*m2*theta1_dot*theta2_dot*sin(theta2) + 2*L1*L2*m3*theta1_dot*theta2_dot*sin(theta2) + 2*L1*L2*m4*theta1_dot*theta2_dot*sin(theta2) + 2*L1*L2*m5*theta1_dot*theta2_dot*sin(theta2) - 2*m3*r1*r3*theta1_dot*theta2_dot*sin(theta2 + theta3) - 2*m3*r1*r3*theta1_dot*theta3_dot*sin(theta2 + theta3) - 2*m3*r1*r3*theta2_dot*theta3_dot*sin(theta2 + theta3) - 2*L1*m2*r2*theta1_dot*theta2_dot*sin(theta2) - 2*L2*m2*r1*theta1_dot*theta2_dot*sin(theta2) - 2*L1*m3*r2*theta1_dot*theta2_dot*sin(theta2) - 2*L2*m3*r1*theta1_dot*theta2_dot*sin(theta2) - 2*L1*m4*r2*theta1_dot*theta2_dot*sin(theta2) - 2*L2*m4*r1*theta1_dot*theta2_dot*sin(theta2) - 2*L1*m5*r2*theta1_dot*theta2_dot*sin(theta2) - 2*L2*m5*r1*theta1_dot*theta2_dot*sin(theta2) + 2*L2*m3*r3*theta1_dot*theta3_dot*sin(theta3) + 2*L2*m3*r3*theta2_dot*theta3_dot*sin(theta3) - 2*L1*m5*r5*theta1_dot*theta2_dot*sin(theta2 + theta3 - theta4 + theta5) - 2*L1*m5*r5*theta1_dot*theta3_dot*sin(theta2 + theta3 - theta4 + theta5) + 2*L1*m5*r5*theta1_dot*theta4_dot*sin(theta2 + theta3 - theta4 + theta5) - 2*L1*m5*r5*theta2_dot*theta3_dot*sin(theta2 + theta3 - theta4 + theta5) - 2*L1*m5*r5*theta1_dot*theta5_dot*sin(theta2 + theta3 - theta4 + theta5) + 2*L1*m5*r5*theta2_dot*theta4_dot*sin(theta2 + theta3 - theta4 + theta5) - 2*L1*m5*r5*theta2_dot*theta5_dot*sin(theta2 + theta3 - theta4 + theta5) + 2*L1*m5*r5*theta3_dot*theta4_dot*sin(theta2 + theta3 - theta4 + theta5) - 2*L1*m5*r5*theta3_dot*theta5_dot*sin(theta2 + theta3 - theta4 + theta5) + 2*L1*m5*r5*theta4_dot*theta5_dot*sin(theta2 + theta3 - theta4 + theta5) + 2*m2*r1*r2*theta1_dot*theta2_dot*sin(theta2) + 2*m3*r1*r2*theta1_dot*theta2_dot*sin(theta2) + 2*m4*r1*r2*theta1_dot*theta2_dot*sin(theta2) + 2*m5*r1*r2*theta1_dot*theta2_dot*sin(theta2) - 2*m3*r2*r3*theta1_dot*theta3_dot*sin(theta3) - 2*m3*r2*r3*theta2_dot*theta3_dot*sin(theta3) + 2*m5*r4*r5*theta1_dot*theta5_dot*sin(theta5) + 2*m5*r4*r5*theta2_dot*theta5_dot*sin(theta5) + 2*m5*r4*r5*theta3_dot*theta5_dot*sin(theta5) - 2*m5*r4*r5*theta4_dot*theta5_dot*sin(theta5) - 2*L2*m4*r4*theta1_dot*theta3_dot*sin(theta3 - theta4) + 2*L2*m4*r4*theta1_dot*theta4_dot*sin(theta3 - theta4) - 2*L2*m4*r4*theta2_dot*theta3_dot*sin(theta3 - theta4) - 2*L2*m5*r4*theta1_dot*theta3_dot*sin(theta3 - theta4) + 2*L2*m4*r4*theta2_dot*theta4_dot*sin(theta3 - theta4) + 2*L2*m5*r4*theta1_dot*theta4_dot*sin(theta3 - theta4) - 2*L2*m5*r4*theta2_dot*theta3_dot*sin(theta3 - theta4) + 2*L2*m4*r4*theta3_dot*theta4_dot*sin(theta3 - theta4) + 2*L2*m5*r4*theta2_dot*theta4_dot*sin(theta3 - theta4) + 2*L2*m5*r4*theta3_dot*theta4_dot*sin(theta3 - theta4) + 2*m5*r1*r5*theta1_dot*theta2_dot*sin(theta2 + theta3 - theta4 + theta5) + 2*m5*r1*r5*theta1_dot*theta3_dot*sin(theta2 + theta3 - theta4 + theta5) - 2*m5*r1*r5*theta1_dot*theta4_dot*sin(theta2 + theta3 - theta4 + theta5) + 2*m5*r1*r5*theta2_dot*theta3_dot*sin(theta2 + theta3 - theta4 + theta5) + 2*m5*r1*r5*theta1_dot*theta5_dot*sin(theta2 + theta3 - theta4 + theta5) - 2*m5*r1*r5*theta2_dot*theta4_dot*sin(theta2 + theta3 - theta4 + theta5) + 2*m5*r1*r5*theta2_dot*theta5_dot*sin(theta2 + theta3 - theta4 + theta5) - 2*m5*r1*r5*theta3_dot*theta4_dot*sin(theta2 + theta3 - theta4 + theta5) + 2*m5*r1*r5*theta3_dot*theta5_dot*sin(theta2 + theta3 - theta4 + theta5) - 2*m5*r1*r5*theta4_dot*theta5_dot*sin(theta2 + theta3 - theta4 + theta5) + 2*m4*r2*r4*theta1_dot*theta3_dot*sin(theta3 - theta4) - 2*m4*r2*r4*theta1_dot*theta4_dot*sin(theta3 - theta4) + 2*m4*r2*r4*theta2_dot*theta3_dot*sin(theta3 - theta4) + 2*m5*r2*r4*theta1_dot*theta3_dot*sin(theta3 - theta4) - 2*m4*r2*r4*theta2_dot*theta4_dot*sin(theta3 - theta4) - 2*m5*r2*r4*theta1_dot*theta4_dot*sin(theta3 - theta4) + 2*m5*r2*r4*theta2_dot*theta3_dot*sin(theta3 - theta4) - 2*m4*r2*r4*theta3_dot*theta4_dot*sin(theta3 - theta4) - 2*m5*r2*r4*theta2_dot*theta4_dot*sin(theta3 - theta4) - 2*m5*r2*r4*theta3_dot*theta4_dot*sin(theta3 - theta4);
RHS(2,1) = u2 + g*m4*(sin(theta1 + theta2)*(L2 - r2) - r4*sin(theta1 + theta2 + theta3 - theta4)) + g*m3*(r3*sin(theta1 + theta2 + theta3) + sin(theta1 + theta2)*(L2 - r2)) - g*m5*(r5*sin(theta1 + theta2 + theta3 - theta4 + theta5) - sin(theta1 + theta2)*(L2 - r2) + r4*sin(theta1 + theta2 + theta3 - theta4)) + g*m2*sin(theta1 + theta2)*(L2 - r2) - m4*r1*r4*theta1_dot^2*sin(theta2 + theta3 - theta4) - m5*r1*r4*theta1_dot^2*sin(theta2 + theta3 - theta4) + m5*r2*r5*theta3_dot^2*sin(theta3 - theta4 + theta5) + m5*r2*r5*theta4_dot^2*sin(theta3 - theta4 + theta5) + m5*r2*r5*theta5_dot^2*sin(theta3 - theta4 + theta5) - L1*m3*r3*theta1_dot^2*sin(theta2 + theta3) - L1*L2*m2*theta1_dot^2*sin(theta2) - L1*L2*m3*theta1_dot^2*sin(theta2) - L1*L2*m4*theta1_dot^2*sin(theta2) - L1*L2*m5*theta1_dot^2*sin(theta2) + m3*r1*r3*theta1_dot^2*sin(theta2 + theta3) + L1*m2*r2*theta1_dot^2*sin(theta2) + L2*m2*r1*theta1_dot^2*sin(theta2) + L1*m3*r2*theta1_dot^2*sin(theta2) + L2*m3*r1*theta1_dot^2*sin(theta2) + L1*m4*r2*theta1_dot^2*sin(theta2) + L2*m4*r1*theta1_dot^2*sin(theta2) + L1*m5*r2*theta1_dot^2*sin(theta2) + L2*m5*r1*theta1_dot^2*sin(theta2) + L2*m3*r3*theta3_dot^2*sin(theta3) + L1*m5*r5*theta1_dot^2*sin(theta2 + theta3 - theta4 + theta5) - m2*r1*r2*theta1_dot^2*sin(theta2) - m3*r1*r2*theta1_dot^2*sin(theta2) - m4*r1*r2*theta1_dot^2*sin(theta2) - m5*r1*r2*theta1_dot^2*sin(theta2) - m3*r2*r3*theta3_dot^2*sin(theta3) + m5*r4*r5*theta5_dot^2*sin(theta5) - L2*m4*r4*theta3_dot^2*sin(theta3 - theta4) - L2*m4*r4*theta4_dot^2*sin(theta3 - theta4) - L2*m5*r4*theta3_dot^2*sin(theta3 - theta4) - L2*m5*r4*theta4_dot^2*sin(theta3 - theta4) - m5*r1*r5*theta1_dot^2*sin(theta2 + theta3 - theta4 + theta5) + m4*r2*r4*theta3_dot^2*sin(theta3 - theta4) + m4*r2*r4*theta4_dot^2*sin(theta3 - theta4) + m5*r2*r4*theta3_dot^2*sin(theta3 - theta4) + m5*r2*r4*theta4_dot^2*sin(theta3 - theta4) + L1*m4*r4*theta1_dot^2*sin(theta2 + theta3 - theta4) + L1*m5*r4*theta1_dot^2*sin(theta2 + theta3 - theta4) - L2*m5*r5*theta3_dot^2*sin(theta3 - theta4 + theta5) - L2*m5*r5*theta4_dot^2*sin(theta3 - theta4 + theta5) - L2*m5*r5*theta5_dot^2*sin(theta3 - theta4 + theta5) - 2*L2*m5*r5*theta1_dot*theta3_dot*sin(theta3 - theta4 + theta5) + 2*L2*m5*r5*theta1_dot*theta4_dot*sin(theta3 - theta4 + theta5) - 2*L2*m5*r5*theta2_dot*theta3_dot*sin(theta3 - theta4 + theta5) - 2*L2*m5*r5*theta1_dot*theta5_dot*sin(theta3 - theta4 + theta5) + 2*L2*m5*r5*theta2_dot*theta4_dot*sin(theta3 - theta4 + theta5) - 2*L2*m5*r5*theta2_dot*theta5_dot*sin(theta3 - theta4 + theta5) + 2*L2*m5*r5*theta3_dot*theta4_dot*sin(theta3 - theta4 + theta5) - 2*L2*m5*r5*theta3_dot*theta5_dot*sin(theta3 - theta4 + theta5) + 2*L2*m5*r5*theta4_dot*theta5_dot*sin(theta3 - theta4 + theta5) + 2*m5*r2*r5*theta1_dot*theta3_dot*sin(theta3 - theta4 + theta5) - 2*m5*r2*r5*theta1_dot*theta4_dot*sin(theta3 - theta4 + theta5) + 2*m5*r2*r5*theta2_dot*theta3_dot*sin(theta3 - theta4 + theta5) + 2*m5*r2*r5*theta1_dot*theta5_dot*sin(theta3 - theta4 + theta5) - 2*m5*r2*r5*theta2_dot*theta4_dot*sin(theta3 - theta4 + theta5) + 2*m5*r2*r5*theta2_dot*theta5_dot*sin(theta3 - theta4 + theta5) - 2*m5*r2*r5*theta3_dot*theta4_dot*sin(theta3 - theta4 + theta5) + 2*m5*r2*r5*theta3_dot*theta5_dot*sin(theta3 - theta4 + theta5) - 2*m5*r2*r5*theta4_dot*theta5_dot*sin(theta3 - theta4 + theta5) + 2*L2*m3*r3*theta1_dot*theta3_dot*sin(theta3) + 2*L2*m3*r3*theta2_dot*theta3_dot*sin(theta3) - 2*m3*r2*r3*theta1_dot*theta3_dot*sin(theta3) - 2*m3*r2*r3*theta2_dot*theta3_dot*sin(theta3) + 2*m5*r4*r5*theta1_dot*theta5_dot*sin(theta5) + 2*m5*r4*r5*theta2_dot*theta5_dot*sin(theta5) + 2*m5*r4*r5*theta3_dot*theta5_dot*sin(theta5) - 2*m5*r4*r5*theta4_dot*theta5_dot*sin(theta5) - 2*L2*m4*r4*theta1_dot*theta3_dot*sin(theta3 - theta4) + 2*L2*m4*r4*theta1_dot*theta4_dot*sin(theta3 - theta4) - 2*L2*m4*r4*theta2_dot*theta3_dot*sin(theta3 - theta4) - 2*L2*m5*r4*theta1_dot*theta3_dot*sin(theta3 - theta4) + 2*L2*m4*r4*theta2_dot*theta4_dot*sin(theta3 - theta4) + 2*L2*m5*r4*theta1_dot*theta4_dot*sin(theta3 - theta4) - 2*L2*m5*r4*theta2_dot*theta3_dot*sin(theta3 - theta4) + 2*L2*m4*r4*theta3_dot*theta4_dot*sin(theta3 - theta4) + 2*L2*m5*r4*theta2_dot*theta4_dot*sin(theta3 - theta4) + 2*L2*m5*r4*theta3_dot*theta4_dot*sin(theta3 - theta4) + 2*m4*r2*r4*theta1_dot*theta3_dot*sin(theta3 - theta4) - 2*m4*r2*r4*theta1_dot*theta4_dot*sin(theta3 - theta4) + 2*m4*r2*r4*theta2_dot*theta3_dot*sin(theta3 - theta4) + 2*m5*r2*r4*theta1_dot*theta3_dot*sin(theta3 - theta4) - 2*m4*r2*r4*theta2_dot*theta4_dot*sin(theta3 - theta4) - 2*m5*r2*r4*theta1_dot*theta4_dot*sin(theta3 - theta4) + 2*m5*r2*r4*theta2_dot*theta3_dot*sin(theta3 - theta4) - 2*m4*r2*r4*theta3_dot*theta4_dot*sin(theta3 - theta4) - 2*m5*r2*r4*theta2_dot*theta4_dot*sin(theta3 - theta4) - 2*m5*r2*r4*theta3_dot*theta4_dot*sin(theta3 - theta4);
RHS(3,1) = u3 - g*m5*r5*sin(theta1 + theta2 + theta3 - theta4 + theta5) - g*m4*r4*sin(theta1 + theta2 + theta3 - theta4) - g*m5*r4*sin(theta1 + theta2 + theta3 - theta4) + g*m3*r3*sin(theta1 + theta2 + theta3) - m4*r1*r4*theta1_dot^2*sin(theta2 + theta3 - theta4) - m5*r1*r4*theta1_dot^2*sin(theta2 + theta3 - theta4) - m5*r2*r5*theta1_dot^2*sin(theta3 - theta4 + theta5) - m5*r2*r5*theta2_dot^2*sin(theta3 - theta4 + theta5) - L1*m3*r3*theta1_dot^2*sin(theta2 + theta3) + m3*r1*r3*theta1_dot^2*sin(theta2 + theta3) - L2*m3*r3*theta1_dot^2*sin(theta3) - L2*m3*r3*theta2_dot^2*sin(theta3) + L1*m5*r5*theta1_dot^2*sin(theta2 + theta3 - theta4 + theta5) + m3*r2*r3*theta1_dot^2*sin(theta3) + m3*r2*r3*theta2_dot^2*sin(theta3) + m5*r4*r5*theta5_dot^2*sin(theta5) + L2*m4*r4*theta1_dot^2*sin(theta3 - theta4) + L2*m4*r4*theta2_dot^2*sin(theta3 - theta4) + L2*m5*r4*theta1_dot^2*sin(theta3 - theta4) + L2*m5*r4*theta2_dot^2*sin(theta3 - theta4) - m5*r1*r5*theta1_dot^2*sin(theta2 + theta3 - theta4 + theta5) - m4*r2*r4*theta1_dot^2*sin(theta3 - theta4) - m4*r2*r4*theta2_dot^2*sin(theta3 - theta4) - m5*r2*r4*theta1_dot^2*sin(theta3 - theta4) - m5*r2*r4*theta2_dot^2*sin(theta3 - theta4) + L1*m4*r4*theta1_dot^2*sin(theta2 + theta3 - theta4) + L1*m5*r4*theta1_dot^2*sin(theta2 + theta3 - theta4) + L2*m5*r5*theta1_dot^2*sin(theta3 - theta4 + theta5) + L2*m5*r5*theta2_dot^2*sin(theta3 - theta4 + theta5) + 2*L2*m5*r5*theta1_dot*theta2_dot*sin(theta3 - theta4 + theta5) - 2*m5*r2*r5*theta1_dot*theta2_dot*sin(theta3 - theta4 + theta5) - 2*L2*m3*r3*theta1_dot*theta2_dot*sin(theta3) + 2*m3*r2*r3*theta1_dot*theta2_dot*sin(theta3) + 2*m5*r4*r5*theta1_dot*theta5_dot*sin(theta5) + 2*m5*r4*r5*theta2_dot*theta5_dot*sin(theta5) + 2*m5*r4*r5*theta3_dot*theta5_dot*sin(theta5) - 2*m5*r4*r5*theta4_dot*theta5_dot*sin(theta5) + 2*L2*m4*r4*theta1_dot*theta2_dot*sin(theta3 - theta4) + 2*L2*m5*r4*theta1_dot*theta2_dot*sin(theta3 - theta4) - 2*m4*r2*r4*theta1_dot*theta2_dot*sin(theta3 - theta4) - 2*m5*r2*r4*theta1_dot*theta2_dot*sin(theta3 - theta4);
RHS(4,:) = u4 + g*m5*r5*sin(theta1 + theta2 + theta3 - theta4 + theta5) + g*m4*r4*sin(theta1 + theta2 + theta3 - theta4) + g*m5*r4*sin(theta1 + theta2 + theta3 - theta4) + m4*r1*r4*theta1_dot^2*sin(theta2 + theta3 - theta4) + m5*r1*r4*theta1_dot^2*sin(theta2 + theta3 - theta4) + m5*r2*r5*theta1_dot^2*sin(theta3 - theta4 + theta5) + m5*r2*r5*theta2_dot^2*sin(theta3 - theta4 + theta5) - L1*m5*r5*theta1_dot^2*sin(theta2 + theta3 - theta4 + theta5) - m5*r4*r5*theta5_dot^2*sin(theta5) - L2*m4*r4*theta1_dot^2*sin(theta3 - theta4) - L2*m4*r4*theta2_dot^2*sin(theta3 - theta4) - L2*m5*r4*theta1_dot^2*sin(theta3 - theta4) - L2*m5*r4*theta2_dot^2*sin(theta3 - theta4) + m5*r1*r5*theta1_dot^2*sin(theta2 + theta3 - theta4 + theta5) + m4*r2*r4*theta1_dot^2*sin(theta3 - theta4) + m4*r2*r4*theta2_dot^2*sin(theta3 - theta4) + m5*r2*r4*theta1_dot^2*sin(theta3 - theta4) + m5*r2*r4*theta2_dot^2*sin(theta3 - theta4) - L1*m4*r4*theta1_dot^2*sin(theta2 + theta3 - theta4) - L1*m5*r4*theta1_dot^2*sin(theta2 + theta3 - theta4) - L2*m5*r5*theta1_dot^2*sin(theta3 - theta4 + theta5) - L2*m5*r5*theta2_dot^2*sin(theta3 - theta4 + theta5) - 2*L2*m5*r5*theta1_dot*theta2_dot*sin(theta3 - theta4 + theta5) + 2*m5*r2*r5*theta1_dot*theta2_dot*sin(theta3 - theta4 + theta5) - 2*m5*r4*r5*theta1_dot*theta5_dot*sin(theta5) - 2*m5*r4*r5*theta2_dot*theta5_dot*sin(theta5) - 2*m5*r4*r5*theta3_dot*theta5_dot*sin(theta5) + 2*m5*r4*r5*theta4_dot*theta5_dot*sin(theta5) - 2*L2*m4*r4*theta1_dot*theta2_dot*sin(theta3 - theta4) - 2*L2*m5*r4*theta1_dot*theta2_dot*sin(theta3 - theta4) + 2*m4*r2*r4*theta1_dot*theta2_dot*sin(theta3 - theta4) + 2*m5*r2*r4*theta1_dot*theta2_dot*sin(theta3 - theta4);
RHS(5,:) = u5 - g*m5*r5*sin(theta1 + theta2 + theta3 - theta4 + theta5) - m5*r2*r5*theta1_dot^2*sin(theta3 - theta4 + theta5) - m5*r2*r5*theta2_dot^2*sin(theta3 - theta4 + theta5) + L1*m5*r5*theta1_dot^2*sin(theta2 + theta3 - theta4 + theta5) - m5*r4*r5*theta1_dot^2*sin(theta5) - m5*r4*r5*theta2_dot^2*sin(theta5) - m5*r4*r5*theta3_dot^2*sin(theta5) - m5*r4*r5*theta4_dot^2*sin(theta5) - m5*r1*r5*theta1_dot^2*sin(theta2 + theta3 - theta4 + theta5) + L2*m5*r5*theta1_dot^2*sin(theta3 - theta4 + theta5) + L2*m5*r5*theta2_dot^2*sin(theta3 - theta4 + theta5) + 2*L2*m5*r5*theta1_dot*theta2_dot*sin(theta3 - theta4 + theta5) - 2*m5*r2*r5*theta1_dot*theta2_dot*sin(theta3 - theta4 + theta5) - 2*m5*r4*r5*theta1_dot*theta2_dot*sin(theta5) - 2*m5*r4*r5*theta1_dot*theta3_dot*sin(theta5) + 2*m5*r4*r5*theta1_dot*theta4_dot*sin(theta5) - 2*m5*r4*r5*theta2_dot*theta3_dot*sin(theta5) + 2*m5*r4*r5*theta2_dot*theta4_dot*sin(theta5) + 2*m5*r4*r5*theta3_dot*theta4_dot*sin(theta5);

end

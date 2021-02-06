%% This is control function:
function [u, x_des] = control(t, state, trajectory, controller)

% Extract state
x1 = state(1); 
x2 = state(2); 
x = [x1, x2];
dx1 = state(3); 
dx2 = state(4); 
dx = [dx1, dx2];

% Extract Sine wave parameters from trajectory
A = trajectory.A ;
A0 = trajectory.A0;
nu = trajectory.nu;
% k = controller.k;
% kd = controller.kd;

xdes1 = controller.xdes1;
xdes2 = controller.xdes2;

m = controller.m;
M = controller.M;
l = controller.l;
g = controller.g;
lambda = controller.lambda;


% CHANGE THIS POSITION DESIRED
x_des(:,1) = xdes1;
x_des(:,2) = xdes2;

dx_des(:,1) = 0;
dx_des(:,2) = 0;

ddx_des(:,1) = 0;
ddx_des(:,2) = 0;

dgdx =[0  0;
        0 -g];
H=[1;0];
D = [M+m   m*l;
    1       l];
d1=-inv(D)*dgdx;

A= [ zeros(2,2) eye(2);
    -d1 zeros(2,2)]

B= [0;0; inv(D)* H];

% Find nonlinear function 
Q = [10 0 0 0;
    0 10 0 0;
    0 0 10 0;
    0 0 0 100];
R = 1;

%%
a=det(ctrb(A,B));
rankA= rank(ctrb(A,B));
%%
K = lqr(A,B,Q,R)

% Extract PD gains

e = -[x_des dx_des] + [x dx];

u =  K*e';


end

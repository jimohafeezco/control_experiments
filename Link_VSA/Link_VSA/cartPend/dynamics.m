%% This is our equation to integrate (solve):
function dstate = dynamics(t, state, system, trajectory, controller)
% We choose following state: state = [x1, x2, dx1, dx2]
% with time derevitive given by: dstate = [dx1, ddx1, dx2, ddx2]

% Extract parameters from struct 'system'
m = system.m; 
M = system.M; 
l = system.l; 
g = system.g; 
% Extract states 
x1 = state(1); 
x2 = state(2); 
dx1 = state(3); 
dx2 = state(4); 
dx = [dx1; dx2];
d11=M+m;
d12=m*l*cos(x2);
d21=cos(x2);
d22=l;
D= [d11 d12;
    d21  d22];
g1= -m*l*(dx2^2)*sin(x2);
g2= -g* sin(x2);
% Find 'Inertia matrix'
% Find nonlinear function 
beta = [g1;g2];
H=[1;0];
[u, x_des] = control(t, state, trajectory, controller);

% Equation for second order derevitive
ddx = D\(H*u - beta);

% Combine dx, ddx back to time derevitive of state
dstate = [dx; ddx];
end
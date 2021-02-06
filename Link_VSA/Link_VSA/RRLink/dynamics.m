%% This is our equation to integrate (solve):
function dstate = dynamics(t, state, system, trajectory, controller)
% We choose following state: state = [x1, x2, dx1, dx2]
% with time derevitive given by: dstate = [dx1, ddx1, dx2, ddx2]

% Extract parameters from struct 'system'
m1 = system.m1; 
m2 = system.m2; 
l1 = system.l2; 
l2 = system.l2; 
I1 = m1 * 0.5* l1^2;
I2 = m2 * 0.5* l2^2;

% Extract states 
x1 = state(1); 
x2 = state(2); 
dx1 = state(3); 
dx2 = state(4); 
dx = [dx1; dx2];
d11=m1* (0.5*l1)^2+ m2*(l1^2+ (0.5* l2)^2+ 2*l1*(0.5* l2)* cos(x2)+ I1+I2);
d12=m2* ((l2*0.5)^2+ (l1* 0.5* l2* cos(x2)))+I2;
d21=m2* ((l2*0.5)^2+ (l1* 0.5* l2* cos(x2)))+I2;
d22=m2* (0.5* l2)^2+ I2;
c11= -m2*l1* 0.5* l2* sin(x2)*dx2;
c12=-m2*l1* 0.5* l2* sin(x2)*(dx1+ dx2);
c21= m2*l1* 0.5* l2* sin(x2)*dx1;
c22= 0;
g1= (m1* 0.5* l1 + m2* l1)* 10 * cos(x1)+m2 * (0.5*l2)* 10 * cos(x1+x2);
g2 =m2 * (0.5*l2)* 10 * cos(x1+x2);
% Find 'Inertia matrix'
D = [d11 d12;
    d21 d22];
% Find nonlinear function 
beta = [c11*dx1+ c12* dx2+g1;
        c21*dx1 + g2];
    
[u, x_des] = control(t, state, trajectory, controller);

% Equation for second order derevitive
ddx = D\(u - beta);

% Combine dx, ddx back to time derevitive of state
dstate = [dx; ddx]; 
end
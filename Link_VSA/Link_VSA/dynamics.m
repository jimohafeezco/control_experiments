%% This is our equation to integrate (solve):
function dstate = dynamics(t, state, system, trajectory, controller)
% We choose following state: state = [x1, x2, dx1, dx2]
% with time derevitive given by: dstate = [dx1, ddx1, dx2, ddx2]

% Extract parameters from struct 'system'
a = system.a; 
b = system.b; 
c = system.c; 
d = system.d; 
k1= system.k1;
k2=system.k2;
% Extract states 
x1 = state(1); 
x2 = state(2); 
x3 = state(3); 
x4 = state(4); 
dx1 = state(5); 
dx2 = state(6); 
dx3 = state(7); 
dx4 = state(8); 


m1 = system.m1;
m2 = system.m2;
l1 = system.l1;
l2 = system.l2;

x = [x1; x2; x3; x4];
dx = [dx1; dx2; dx3; dx4];



I1 = m1 * 0.5* l1^2;
I2 = m2 * 0.5* l2^2;


c11= -m2*l1* 0.5* l2* sin(x2)*dx2;
c12=-m2*l1* 0.5* l2* sin(x2)*(dx1+ dx2);
c21= m2*l1* 0.5* l2* sin(x2)*dx1;
c22= 0;

d11=m1* (0.5*l1)^2+ m2*(l1^2+ (0.5* l2)^2+ 2*l1*(0.5* l2)* cos(x2)+ I1+I2);
d12=m2* ((l2*0.5)^2+ (l1* 0.5* l2* cos(x2)))+I2;
d21=m2* ((l2*0.5)^2+ (l1* 0.5* l2* cos(x2)))+I2;
d22=m2* (0.5* l2)^2+ I2;

d1=1*10^-5;
d2=1*10^-5;

g1= (m1* 0.5* l1 + m2* l1)* 10 * cos(x1)+m2 * (0.5*l2)* 10 * cos(x1+x2);
g2 =m2 * (0.5*l2)* 10 * cos(x1+x2);


D = [d11 d12 0 0;
    d21 d22 0 0;
    0 0  d1  0;
    0  0  0  d2];

C = [c11 c12 0 0;
     c21 0 0 0;
     0 0 0 0
     0 0 0 0];

G =[g1+(x1-x3); g2+ (x2-x4); k1*(x3-x1); k2*(x4-x2)];
beta = C*dx+G;




% Control input implimented as external function of (
[u, x_des] = control(t, state, trajectory, controller);

% Equation for second order derevitive
ddx = D\(u' - beta);

% Combine dx, ddx back to time derevitive of state
dstate = [dx; ddx];
end
% Combine dx, ddx back to time derevitive of state

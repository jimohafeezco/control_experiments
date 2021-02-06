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
kp = controller.kp;
kd = controller.kd;

xdes1 = controller.xdes1;
xdes2 = controller.xdes2;

m1 = controller.m1;
m2 = controller.m2;
l1 = controller.l1;
l2 = controller.l2;
lambda = controller.lambda;


% CHANGE THIS POSITION DESIRED
x_des(:,1) = xdes1;
x_des(:,2) = xdes2;

dx_des(:,1) = 0;
dx_des(:,2) = 0;

ddx_des(:,1) = 0;
ddx_des(:,2) = 0;

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
g1= (m1* 0.5* l1 + m2* l1)* 10 * cos(x1)+m2 * (0.5*l2)* 10 * cos(x1+x2);
g2 =m2 * (0.5*l2)* 10 * cos(x1+x2);
% Find 'Inertia matrix'
D = [d11 d12;
    d21 d22];
% Find nonlinear function 
beta = [c11*dx1+ c12* dx2+g1;
        c21*dx1 + g2];


e = x_des - x;
de = dx_des - dx;
    
% 
% z = de' +lambda *e';
% rho=10;
% eps =0.1;
% if norm(z)== eps
%    w =rho*z/norm(z);
% else   
%    w = rho*z/norm(z); 
% end
   
    
        
% Extract PD gains


u_star =  kp*e' + kd*de'; % 1
% u_star = kp*e' + kd*de' + ddx_des' +w;    

% u=[4,5]';
% CHANGE THIS CONTROL LAWS
%u = D*u_star1; % 1
%u = D*u_star2 + beta; % 2
% u = D*u+ beta; % 2
u = D*u_star+ beta; % 2


end

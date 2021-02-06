function dstate = dynamics(state)
x1 = state(1); 
x2 = state(2); 
% x3 = state(3); 
% x4 = state(4); 
% x=[x1; x2];
dx1 = state(3); 
dx2 = state(4); 
dx = [dx1; dx2];

m1=1;
m2=1;
l1=0.5;
l2=0.5;
lc1=0.5*l1;
lc2=0.5*l2;
B=5;
g=9.81;
I1=0.008;
I2=0.33;


c1=m1*(lc1^2)+m2*l1^2+I1;
c2=m2*lc2^2+I2;
c3=m2*l1*lc2;

d11 = c1+c2+2*c3*cos(x2);
d12= c2+c3*cos(x2);
d21=d12;
d22=c2;

h=-m2*l1* 0.5* l2* sin(x2);
c11=h *dx2;
c12=h *dx2+ h*dx1;
c21= -h *dx1;
c22= 0;
g1= (m1* 0.5* l1 + m2* l1)* g * cos(x1)+ m2 * (0.5*l2)* g * cos(x1+x2);
g2 =m2 * (0.5*l2)* g * cos(x1+x2);
% Find 'Inertia matrix'
D = [d11 d12;
    d21 d22];

G=[g1;g2];
C=[c11+c12; c21+c22];

% u = sym('u', [2, 1]); assume(u, 'real');
% % 
% k = sym('k', [2, 1]); assume(k, 'real');
% % 
% % 
% D2 = eye(2)*1;
% 
% 
% u_springs = [k(1)*(q_theta(1) - x1);
%              k(1)*(x1+q_theta(2) - x2)];
% 
% k=1;
% K = diag(k);
% LHS = [K*(q_theta - q) - C;
%        u(1:2) - K*(q_theta - q)];
%    

% F_elastic = K*[(x3-x1);
%                (x4-x2)];

beta  =C+G;
    
% D = blkdiag(D, D2);
% H=[0 0; 0 0; 1 0; 0 1];
H=[0;1];
% u=1;
[u, ~] = control(state);

ddx = D\(H*u - beta);
dstate =ddx;

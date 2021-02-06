function u= control(t, state, controller)

m1= controller.m1;

m2= controller.m2;
l1= controller.l1;
l2= controller.l2;
g= controller.g;
I1= controller.I1;
I2= controller.I2;
x_des= controller.x_des;
x1= state(1);
x2= state(2);
dx1= state(3);
dx2= state(4);
% I1 = m1 * 0.5* l1^2;
% I2 = m2 * 0.5* l2^2;

x= [x1; x2];
dx=[dx1; dx2];

% d11=m1* (0.5*l1)^2+ m2*(l1^2+ (0.5*l2)^2)+I1+I2;
% d12=m2* ((l2*0.5)^2)+I2;
% d21=d12;
% d22=m2* (0.5* l2)^2+ I2;

% Find 'Inertia matrix'
% D = [d11 d12;
%     d21 d22];
% % dgdx= [ 0 1;
%      0 0];
H = [0;1];
lc1=0.5*l1;
lc2=0.5*l2;

% d1=-inv(D)*dgdx;
c1=m1*(lc1^2)+m2*l1^2+I1;
c2=m2*lc2^2+I2;
c3=m2*l1*lc2;
k1=(m1*lc1+m2*l1)*g;
k2=m2*lc2*g;
% kx=[k1+k2    k2; k2     k2];
d=[c1+c2+2*c3   c2+c3;
    c2+c3              c2];
kx=[0 0 ; 0 0];
A= [ zeros(2,2) eye(2);
    -inv(d)*kx  zeros(2,2)];
H=[0 ; 1];
B= [0;0; inv(d)*H];

% 
% B = [0; 1/M; 0; s*1/(M*L)];
% eig(A)

Q = [1 0 0 0;
    0 6 0 0;
    0 0 10 0;
    0 0 0 6];

R = 1;
Kp=10;
kd=2;
%%
det(ctrb(A,B));
%%
% K = lqr(A,B,Q,R);
% x_des=[pi/2; 0; 0; 0];
e = x;
% u=-K*([x1; x2; dx1; dx2]-x_des)
u=0;
end
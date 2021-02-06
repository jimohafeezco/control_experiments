function ddx = dynamics(t,y,controller)

m1= controller.m1;
m2= controller.m2;
l1= controller.l1;
l2= controller.l2;
I1= controller.I1;
I2= controller.I2;
g= controller.g;
lc1=l1*0.5;
lc2=l2*0.5;

x1= y(1);
x2= y(2);
dx1= y(3);
dx2= y(4);

I1 = m1 * 0.5* l1^2;
I2 = m2 * 0.5* l2^2;
% 
% d11=m1* (0.5*l1)^2+ m2*(l1^2+ (0.5* l2)^2+ 2*l1*(0.5* l2)* cos(x2)+ I1+I2);
% d12=m2* ((l2*0.5)^2+ l1* 0.5* l2* cos(x2))+I2;
% d21=m2* ((l2*0.5)^2+ (l1* 0.5* l2* cos(x2)))+I2;
% d22=m2* (0.5* l2)^2+ I2;
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
g1= (m1* 0.5* l1 + m2* l1)* g * cos(x1)+m2 * (0.5*l2)* g * cos(x1+x2);
g2 =m2 * (0.5*l2)* g * cos(x1+x2);
% Find 'Inertia matrix'
D = [d11 d12;
    d21 d22];
% Find nonlinear function 
beta = [c11*dx1+ c12* dx2+g1;
        c21*dx1 + g2];
H=[1; 1] ;  
% [u, x_des] = control(t, y, trajectory, controller);
u= control(t, y, controller);
% Equation for second order derevitive
ddx1 = D\(H*u - beta);
ddx = [dx1; dx2; ddx1]; 

end
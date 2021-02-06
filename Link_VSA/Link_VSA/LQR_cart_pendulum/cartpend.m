function dy = cartpend(y,m,M,l,g,d,u)
% 
% Sy = sin(y(3));
% Cy = cos(y(3));
% D = m*L*L*(M+m*(1-Cy^2));
% 
% dy(1,1) = y(2);
% dy(2,1) = (1/D)*(-m^2*L^2*g*Cy*Sy + m*L^2*(m*L*y(4)^2*Sy - d*y(2))) + m*L*L*(1/D)*u;
% dy(3,1) = y(4);
% dy(4,1) = (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*y(4)^2*Sy - d*y(2))) - m*L*Cy*(1/D)*u +.01*randn;
d11=M+m;
d12=m*l*cos(y(2));
d21=cos(y(2));
d22=l;
D= [d11 d12;
    d21  d22];
g1= -m*l*(y(4)^2)*sin(y(2));
g2= -g* sin(y(2));


beta = [g1;g2];
H=[1;0];
% [u, x_des] = control(t, state, trajectory, controller);

% Equation for second order derevitive
dy = D\(H*u - beta);
dy = [y(3); y(4);dy];
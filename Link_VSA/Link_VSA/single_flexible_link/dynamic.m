function ds = dynamic(in1,u1)
%DYNAMIC
%    DS = DYNAMIC(IN1,U1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    23-May-2020 19:44:45

dx1 = in1(3,:);
dx2 = in1(4,:);
x1 = in1(1,:);
x2 = in1(2,:);
t2 = x2.*1.0e4;
ds = [dx1;dx2;t2-x1.*1.0e4-sin(x1).*(9.81e2./1.0e1);dx2.*(-1.0./2.0)-t2+u1.*1.0e1+x1.*1.0e4];

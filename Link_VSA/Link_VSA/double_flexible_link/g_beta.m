function beta = g_beta(in1)
%G_BETA
%    BETA = G_BETA(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    27-May-2020 19:15:23

dx1 = in1(5,:);
dx2 = in1(6,:);
x1 = in1(1,:);
x2 = in1(2,:);
x3 = in1(3,:);
x4 = in1(4,:);
t2 = sin(x2);
t3 = x1+x2;
t4 = cos(t3);
t5 = t4.*(9.81e2./4.0e2);
t6 = x1.*1.0e3;
t7 = x2.*1.0e3;
beta = [t5+t6-x3.*1.0e3+cos(x1).*7.3575-(dx1.*t2)./8.0-(dx2.*t2)./4.0;t5+t7-x4.*1.0e3+(dx1.*t2)./8.0;-t6+x3.*1.002e3;-t7+x4.*1.002e3];

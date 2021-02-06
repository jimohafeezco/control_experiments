function C = g_Linear_C(in1)
%G_LINEAR_C
%    C = G_LINEAR_C(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    08-Jun-2020 20:23:54

dx1 = in1(7,:);
dx2 = in1(8,:);
k1 = in1(5,:);
k2 = in1(6,:);
x1 = in1(1,:);
x2 = in1(2,:);
x3 = in1(3,:);
x4 = in1(4,:);
t2 = x2.*2.0;
t3 = sin(x1);
t4 = -t2+x1;
t5 = t2+x1;
t6 = sin(t4);
t7 = sin(t5);
t8 = x2.*4.0;
t9 = t8+x1;
t10 = cos(x2);
t11 = cos(t2);
t12 = x2.*3.0;
t13 = cos(t12);
t14 = x2.^2;
t15 = sin(t2);
t16 = sin(t12);
t17 = sin(x2);
t18 = t11.*2.0;
t19 = t18-3.55e2;
t20 = 1.0./t19.^2;
t21 = cos(x1);
t22 = t21.*1.7413731e7;
t23 = k1.*x3.*2.414e6;
t24 = t3.*x1.*1.7413731e7;
t25 = t3.*x2.*3.924e3;
t26 = dx2.*t10.*x2.*5.984e5;
t27 = dx2.*t13.*x2.*1.7e3;
t28 = k1.*t11.*x1.*1.36e4;
t29 = cos(t8);
t30 = sin(t8);
t31 = k1.*t15.*x1.*x2.*2.72e4;
C = [0.0;0.0;0.0;0.0;t20.*(t22+t23+t24+t25+t26+t27+t28+t31-cos(t4).*4.905e4-cos(t5).*3.97305e5+cos(t9).*9.81e2-dx1.*x2.*2.0e2-k1.*x1.*2.414e6+k2.*x2.*2.414e6-k2.*x4.*2.414e6-t6.*x1.*4.905e4-t6.*x2.*9.81e4-t7.*x1.*3.97305e5-t7.*x2.*5.9841e5+x1.*sin(t9).*9.81e2+dx1.*t10.*x2.*5.984e5+dx1.*t11.*x2.*3.55e4+dx1.*t13.*x2.*1.7e3-k2.*t14.*t15.*2.72e4-k2.*t14.*t16.*8.0e2-k2.*t14.*t17.*2.864e5+k2.*t10.*x2.*2.832e5-k1.*t11.*x3.*1.36e4-k2.*t11.*x2.*1.36e4-k2.*t10.*x4.*2.832e5+k2.*t11.*x4.*1.36e4-k2.*t13.*x2.*8.0e2+k2.*t13.*x4.*8.0e2-k1.*t15.*x2.*x3.*2.72e4+k2.*t15.*x2.*x4.*2.72e4+k2.*t16.*x2.*x4.*8.0e2+k2.*t17.*x2.*x4.*2.864e5).*(-1.0./2.5e1);(t20.*(t22+t23+t24+t25+t26+t27+t28+t31-dx1.*x2.*6.0e2-dx2.*x2.*4.0e2-k1.*x1.*2.414e6+k2.*x2.*5.396e6-k2.*x4.*5.396e6+t3.*t15.*3.48255e5-t3.*t16.*2.0601e4+t3.*t17.*7.333956e6-t10.*t21.*5.20911e6-t11.*t21.*4.46355e5-t3.*t30.*9.81e2+t13.*t21.*1.4715e4+t21.*t29.*9.81e2+dx1.*t10.*x2.*9.68e5+dx1.*t11.*x2.*1.065e5+dx2.*t11.*x2.*7.1e4+dx1.*t13.*x2.*2.75e3-k2.*t14.*t15.*6.08e4-k2.*t14.*t16.*1.6e3-k2.*t14.*t17.*5.728e5-k1.*t10.*x1.*2.832e5+k1.*t10.*x3.*2.832e5+k2.*t10.*x2.*5.664e5-k1.*t11.*x3.*1.36e4+k1.*t13.*x1.*8.0e2-k2.*t11.*x2.*3.04e4-k2.*t10.*x4.*5.664e5-k1.*t13.*x3.*8.0e2+k2.*t11.*x4.*3.04e4-k2.*t13.*x2.*1.6e3+k2.*t13.*x4.*1.6e3-t3.*t10.*x1.*5.20911e6-t3.*t10.*x2.*7.251552e6-t3.*t11.*x1.*4.46355e5-t3.*t11.*x2.*6.9651e5+t3.*t13.*x1.*1.4715e4-t3.*t13.*x2.*2.0601e4+t3.*t29.*x1.*9.81e2-t15.*t21.*x1.*3.48255e5-t15.*t21.*x2.*5.0031e5+t16.*t21.*x1.*2.0601e4-t16.*t21.*x2.*1.4715e4-t17.*t21.*x1.*7.333956e6-t17.*t21.*x2.*5.26797e6+t21.*t30.*x1.*9.81e2+k1.*t16.*x1.*x2.*8.0e2-k1.*t15.*x2.*x3.*2.72e4+k1.*t17.*x1.*x2.*2.864e5-k1.*t16.*x2.*x3.*8.0e2-k1.*t17.*x2.*x3.*2.864e5+k2.*t15.*x2.*x4.*6.08e4+k2.*t16.*x2.*x4.*1.6e3+k2.*t17.*x2.*x4.*5.728e5))./2.5e1;-k1.*(x1-x3);-k2.*(x2-x4)];

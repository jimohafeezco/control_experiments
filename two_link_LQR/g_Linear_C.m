function C = g_Linear_C(in1,in2)
%G_LINEAR_C
%    C = G_LINEAR_C(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    19-Dec-2020 11:07:32

q1 = in1(1,:);
q2 = in1(2,:);
v1 = in1(3,:);
v2 = in1(4,:);
t2 = q1-q2;
t3 = sin(t2);
t4 = v1.^2;
t5 = v2.^2;
t6 = q1.*2.0;
t8 = q2.*2.0;
t7 = t6-t8;
t9 = sin(t7);
t10 = q1.*3.0;
t12 = q2.*3.0;
t11 = t10-t12;
t13 = sin(t11);
t14 = q1.*4.0;
t20 = q2.*4.0;
t15 = t14-t20;
t16 = cos(t7);
t17 = cos(t11);
t18 = cos(t2);
t19 = t17.*2.16e2;
t21 = cos(t15);
t22 = t21.*1.62e2;
t23 = t16.*-1.692e3-t18.*9.6e2+t19+t22+2.635e3;
t24 = 1.0./t23;
t25 = sin(t15);
C = [0.0;0.0;-t24.*(q1.*t4.*7.2e2+q1.*t5.*2.88e2-q2.*t4.*7.2e2-q2.*t5.*2.88e2-t3.*t4.*2.4e2-t3.*t5.*1.392e3-t4.*t9.*8.46e2+t5.*t9.*1.44e2+t4.*t13.*1.62e2+t5.*t13.*2.16e2+t4.*t25.*1.62e2-q1.*t4.*t16.*1.764e3+q1.*t4.*t17.*5.4e1+q2.*t4.*t16.*1.764e3+q1.*t4.*t18.*1.92e2-q1.*t5.*t17.*2.16e2-q2.*t4.*t17.*5.4e1-q1.*t5.*t18.*5.28e2-q2.*t4.*t18.*1.92e2+q2.*t5.*t17.*2.16e2+q2.*t5.*t18.*5.28e2);t24.*(q1.*t4.*1.224e3+q1.*t5.*7.2e2-q2.*t4.*1.224e3-q2.*t5.*7.2e2-t3.*t4.*5.916e3-t3.*t5.*2.4e2+t4.*t9.*6.12e2-t5.*t9.*8.46e2+t4.*t13.*9.18e2+t5.*t13.*1.62e2+t5.*t25.*1.62e2-q1.*t4.*t17.*9.18e2-q1.*t5.*t16.*1.764e3-q1.*t4.*t18.*2.244e3+q1.*t5.*t17.*5.4e1+q2.*t4.*t17.*9.18e2+q2.*t5.*t16.*1.764e3+q1.*t5.*t18.*1.92e2+q2.*t4.*t18.*2.244e3-q2.*t5.*t17.*5.4e1-q2.*t5.*t18.*1.92e2)];

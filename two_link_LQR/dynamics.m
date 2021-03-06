function ds = dynamics(in1,in2)
%DYNAMICS
%    DS = DYNAMICS(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    19-Dec-2020 11:07:32

q1 = in1(1,:);
q2 = in1(2,:);
u1 = in2(1,:);
u2 = in2(2,:);
v1 = in1(3,:);
v2 = in1(4,:);
t2 = q1-q2;
t3 = sin(t2);
t4 = v1.^2;
t5 = q1.*2.0;
t9 = q2.*2.0;
t6 = t5-t9;
t7 = cos(t2);
t8 = t7.*1.2e1;
t10 = cos(t6);
t11 = t10.*1.8e1;
t12 = t8+t11-4.9e1;
t13 = 1.0./t12;
t14 = v2.^2;
t15 = sin(t6);
ds = [v1;v2;t13.*(u1.*-8.0+u2.*2.0+t3.*t4+t3.*t14.*4.0+t4.*t15.*3.0+t7.*u2.*1.2e1).*6.0;t13.*(u1.*-2.0+u2.*3.4e1+t3.*t4.*1.7e1+t3.*t14+t14.*t15.*3.0-t7.*u1.*1.2e1).*-6.0];

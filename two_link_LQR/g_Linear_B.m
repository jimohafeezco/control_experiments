function B = g_Linear_B(in1)
%G_LINEAR_B
%    B = G_LINEAR_B(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    19-Dec-2020 11:07:32

q1 = in1(1,:);
q2 = in1(2,:);
t2 = q1-q2;
t3 = cos(t2);
t4 = t3.*1.2e1;
t5 = q1.*2.0;
t12 = q2.*2.0;
t6 = t5-t12;
t7 = cos(t6);
t8 = t7.*1.8e1;
t9 = t4+t8-4.9e1;
t10 = 1.0./t9;
t11 = t4+2.0;
t13 = t10.*t11.*6.0;
B = reshape([0.0,0.0,t10.*-4.8e1,t13,0.0,0.0,t13,t10.*-2.04e2],[4,2]);

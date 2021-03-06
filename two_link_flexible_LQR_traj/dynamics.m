function LHS = dynamics(in1,in2)
%DYNAMICS
%    LHS = DYNAMICS(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    19-Dec-2020 09:58:50

q1 = in1(1,:);
q2 = in1(2,:);
q_theta1 = in1(3,:);
q_theta2 = in1(4,:);
u1 = in2(1,:);
u2 = in2(2,:);
v1 = in1(5,:);
v2 = in1(6,:);
v_theta1 = in1(7,:);
v_theta2 = in1(8,:);
t3 = q1-q2;
t2 = cos(t3);
t4 = sin(t3);
t5 = t2.^2;
t6 = t5.*9.0;
t7 = t6-1.6e1;
t8 = 1.0./t7;
t9 = v1.^2;
t10 = v2.^2;
LHS = [v1;v2;v_theta1;v_theta2;t8.*(q1.*8.0e3-q_theta1.*8.0e3-q2.*t2.*1.2e4+q_theta2.*t2.*1.2e4+t4.*t10.*2.0+t2.*t4.*t9.*3.0).*3.0;t8.*(q2.*-3.2e4+q_theta2.*3.2e4+q1.*t2.*1.2e4-q_theta1.*t2.*1.2e4+t4.*t9.*8.0+t2.*t4.*t10.*3.0).*-3.0;q1.*1.0e4-q_theta1.*1.0e4+u1;q2.*1.0e4-q_theta2.*1.0e4+u2];

function LHS = g_LHS(in1,in2)
%G_LHS
%    LHS = G_LHS(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    14-Oct-2020 21:21:27

k1 = in1(9,:);
k2 = in1(10,:);
q1 = in1(5,:);
q2 = in1(6,:);
q_theta1 = in1(7,:);
q_theta2 = in1(8,:);
u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
v1 = in1(1,:);
v2 = in1(2,:);
v_theta1 = in1(3,:);
v_theta2 = in1(4,:);
t2 = q2.*2.0;
t3 = cos(q2);
t4 = sin(q2);
t5 = cos(q1);
t6 = v2.^2;
t7 = sin(t2);
t8 = q1+q2;
t9 = cos(t8);
LHS = [(t5.*1.11834e3-cos(q1+t2).*5.2974e2+k1.*q1.*(9.6e1./5.0)-k2.*q2.*(9.6e1./5.0)-k1.*q_theta1.*(9.6e1./5.0)+k2.*q_theta2.*(9.6e1./5.0)-t4.*t6.*1.8e1-k2.*q2.*t3.*(1.44e2./5.0)+k2.*q_theta2.*t3.*(1.44e2./5.0)-t4.*v1.*v2.*1.8e1+t7.*v1.*v2.*(2.7e1./2.0))./(cos(t2).*2.7e1-5.3e1);(t5.*(-8.2404e2)+t9.*1.1772e3-k1.*q1.*(4.8e1./5.0)+k2.*q2.*(2.08e2./5.0)+k1.*q_theta1.*(4.8e1./5.0)-k2.*q_theta2.*(2.08e2./5.0)-t3.*t5.*1.23606e3+t4.*t6.*9.0+t3.*t9.*5.2974e2+t6.*t7.*(2.7e1./4.0)-k1.*q1.*t3.*(7.2e1./5.0)+k2.*q2.*t3.*(1.44e2./5.0)+k1.*q_theta1.*t3.*(7.2e1./5.0)-k2.*q_theta2.*t3.*(1.44e2./5.0)-t4.*v1.*v2.*2.1e1)./(t3.^2.*2.7e1-4.0e1);u1+k1.*q1-k1.*q_theta1;u2+k2.*q2-k2.*q_theta2;v1;v2;v_theta1;v_theta2;u3;u4];

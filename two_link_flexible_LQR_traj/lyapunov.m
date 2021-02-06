function energy = lyapunov(in1)
%LYAPUNOV
%    ENERGY = LYAPUNOV(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    29-Oct-2020 19:20:42

q1 = in1(1,:);
q2 = in1(2,:);
v1 = in1(3,:);
v2 = in1(4,:);
t2 = cos(q2);
t3 = t2.*(1.0./2.0);
t4 = t3+1.0./3.0;
t5 = v2.*(1.0./3.0);
t6 = t4.*v1;
t7 = t5+t6;
t8 = t7.*v2;
t9 = t2+5.0./3.0;
t10 = t9.*v1;
t11 = t4.*v2;
t12 = t10+t11;
t13 = t12.*v1;
energy = [t8+t13+cos(q1).*1.5e1;t2.*5.0+t8+t13];

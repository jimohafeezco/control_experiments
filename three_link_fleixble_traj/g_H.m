function H = g_H(in1)
%G_H
%    H = G_H(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    19-Dec-2020 09:11:22

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = q1-q2;
t3 = cos(t2);
t4 = t3.*(1.5e1./2.0);
t5 = q1-q3;
t6 = cos(t5);
t7 = t6.*(5.0./2.0);
t8 = q2-q3;
t9 = cos(t8);
t10 = t9.*(5.0./2.0);
H = reshape([3.5e1./3.0,t4,t7,t4,2.0e1./3.0,t10,t7,t10,5.0./3.0],[3,3]);

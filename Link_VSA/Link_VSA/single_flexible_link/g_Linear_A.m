function A = g_Linear_A(in1)
%G_LINEAR_A
%    A = G_LINEAR_A(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    23-May-2020 19:44:46

x1 = in1(1,:);
A = reshape([0.0,0.0,cos(x1).*(-9.81e2./1.0e1)-1.0e4,1.0e4,0.0,0.0,1.0e4,-1.0e4,1.0,0.0,0.0,0.0,0.0,1.0,0.0,-1.0./2.0],[4,4]);
Q = eye(8);
R = eye(2);

for i = 1:10000
    s = rand(8, 1);
    u = rand(2, 1);
    A = g_Linear_A(s);
    B = g_Linear_B(s);
    K = lqr(A, B, Q, R);
    K
end

















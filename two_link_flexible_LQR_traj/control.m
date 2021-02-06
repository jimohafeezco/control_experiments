function u = control(state, x_des)
        
    s= state;
    A = g_Linear_A(s);
    Q = 1000*diag([1; 1; 1; 1; 0.01; 0.01; 0.01; 0.01]);
    R = eye(2)*0.5;

    B = g_Linear_B(s);
    K = lqr(A, B, Q, R);

    rank(ctrb(A,B));
    C = g_Linear_C(s);

    u0 = pinv(B) * (-A*s - C);

    e=s-x_des;
    u= -K*e+u0;
end

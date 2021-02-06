function u = control_funct(state, x_des)
        
    s= state;
    A = g_Linear_A(s);
    Q = 3000*diag([1; 1; 1; 1;1;1; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5]);
%     Q = 1000*diag([1; 1; 1; 1;1;1; 0.1; 0.1; 0.1; 0.1; 0.1; 0.1]);

    %     Q= 1000*ones(12);

    R = eye(3)*0.1;

    B = g_Linear_B(s);
    K = lqr(A, B, Q, R);

    rank(ctrb(A,B));
    C = g_Linear_C(s);

    u0 = pinv(B) * (-A*s - C);

    e=s-x_des;
    u= -K*e+u0;
end

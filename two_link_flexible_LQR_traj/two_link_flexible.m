% Normal dynamics is: 
% 
% H*ddq + c = u
% 
% SEA dynamics is:
% 
% H1*ddq + c = K*(theta - q)
% H2*ddtheta = u - K*(theta - q)
% 
% VSA dynamics is only different in that K is not a constant

m = [5; 5];
L = [1; 1];
g = 9.81;

I = [(1/12)*m(1)*L(1)^2; (1/12)*m(2)*L(2)^2];

q = sym('q', [2, 1]); assume(q, 'real');
v = sym('v', [2, 1]); assume(v, 'real');

Math = MathClass;
Math.UseParallel = false;

rC1 = [0.5*L(1)*cos(q(1)); 0.5*L(1)*sin(q(1)); 0];
rO1 = [    L(1)*cos(q(1));     L(1)*sin(q(1)); 0];

rC2 = rO1 + [0.5*L(2)*cos(q(2)); 
             0.5*L(2)*sin( q(2));
             0];
rO2 = rO1 + [L(2)*cos(q(2)); 
             L(2)*sin(q(2));
             0];
         
         
JC1 = jacobian(rC1, q);

JC2 = jacobian(rC2, q);

JC1 = simplify(JC1);
JC2 = simplify(JC2);

Je = jacobian(rO2,q)

omega1 = [0; 0; v(1)];
omega2 = [0; 0; v(2)];

Jw1 = jacobian(omega1, v);
Jw2 = jacobian(omega2, v);

H = JC1' * m(1) * JC1 + ...
    JC2' * m(2) * JC2 + ...
    Jw1' * I(1) * Jw1 + ...
    Jw2' * I(2) * Jw2;

H = simplify(H);

c1 = 0.5*(jacobian(H(:,1),q)+jacobian(H(:,1),q)'- diff(H,q(1)));
c2 = 0.5*( jacobian(H(:,2),q)+jacobian(H(:,2),q)'- diff(H,q(2)));

c= [v'*c1*v; v'*c2*v];


% 
% dH = Math.MatrixDerivative(H, q, v);
% 
% G = JC1' * [0; -m(1)*g; 0] + ...
%     JC2' * [0; -m(2)*g; 0];
% 
% c = 0.5*dH*v - G;


matlabFunction(H, 'File', 'g_H', 'Vars', {q});
matlabFunction(c, 'File', 'g_c', 'Vars', {q, v});
matlabFunction(rO2, 'File', 'g_rO2', 'Vars', {q});

%%%%%%%%%%%%%%%%%%%%%%%%%

q_theta = sym('q_theta', [2, 1]); assume(q_theta, 'real');
v_theta = sym('v_theta', [2, 1]); assume(v_theta, 'real');

u = sym('u', [2, 1]); assume(u, 'real');

% k = sym('k', [2, 1]); assume(k, 'real');
k = 10000;

H2 = eye(2)*1;

% u_springs = [k(1)*(q_theta(1) - q(1));
%              k(1)*(q(1)+q_theta(2) - q(2))];

K = diag(k);
LHS = [K*(q_theta - q) - c;
       u - K*(q_theta - q)];
   
RHS = blkdiag(H, H2);
LHS = RHS \ LHS;
LHS = simplify(LHS);

LHS = [v; v_theta;LHS];

x = [q; q_theta;v; v_theta];

A = jacobian(LHS, x); 
A = simplify(A);

B = jacobian(LHS, u); 
B = simplify(B);

C = LHS - A*x - B*u;
C = simplify(C); 

Je = jacobian(rO2,q);

Jedot = diff(Je);

G = [Je zeros(2,2);
    zeros(2,4);
    Jedot  Je;
    zeros(2,4)];


G = [Je zeros(2,2);
    zeros(2,4);
    Jedot  Je;
    zeros(2,4)];



matlabFunction(G, 'File', 'g_constraint', 'Vars', {x});

matlabFunction(A, 'File', 'g_Linear_A', 'Vars', {x});
matlabFunction(B, 'File', 'g_Linear_B', 'Vars', {x});
matlabFunction(C, 'File', 'g_Linear_C', 'Vars', {x});
matlabFunction(LHS, 'File', 'dynamics', 'Vars', {x, u});

save('parameters', 'L', 'H2');

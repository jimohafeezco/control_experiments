syms l1 l2 m1 m2 g I1 I2 real
m = [m1; m2];
L = [l1; l2];
% g = 9.81;
% note to self
% remember to replace all m1 m2 l1 l2 and Inertai by their numeric value
I = [(1/12)*m(1)*L(1)^2; (1/12)*m(2)*L(2)^2];
% I= [I1; I2];
q = sym('q', [3, 1]); assume(q, 'real');
v = sym('v', [3, 1]); assume(v, 'real');


p1 = [q(1)+0.5*L(1)* cos(q(2));
    0.5*L(1)* sin(q(2));
    0];
         
p2 = [q(1)+L(1)* cos(q(2))+0.5*L(2) * cos(q(2)+q(3));
    L(1)* sin(q(2))+0.5*L(2) * sin(q(2)+q(3));
    0];

JC1 = simplify(jacobian(p1, q));
JC2 = simplify(jacobian(p2, q));

omega1 = [v(1); 0; v(2)];
omega2 = [0; 0; v(2)+v(3)];
% 
Jw1 = jacobian(omega1, v)
Jw2 = jacobian(omega2, v)
% 
H = JC1' * m(1) * JC1 + ...
    JC2' * m(2) * JC2 + ...
    Jw1' * I(1) * Jw1 + ...
    Jw2' * I(2) * Jw2;
% 
B = simplify(H)
% 
b1 = B(1,:)';
b2 = B(2,:)';
b3 = B(3,:)';

diff_b1= jacobian(b1,q);
diff_b2= jacobian(b2,q);

% diff_B1 = diff(B, q(1));
% diff_B2 = diff(B, q(2));
% 
c1 = 0.5*(diff_b1 + diff_b1'-diff_B1);
c2 = 0.5*(diff_b2 + diff_b2'-diff_B2);
C1 = v' * c1 *v;
C2 = v' * c2 *v;

c = [C1; C2]
% 
V = m1 * g* L(1)/2*sin(q(2))+ m2* g* (L(1)* sin(q(2))+ L(2)/2* sin(q(2)+q(3)));
% 
G = [diff(V, q(2)); diff(V, q(3))]
% Jc = simplify(jaco)
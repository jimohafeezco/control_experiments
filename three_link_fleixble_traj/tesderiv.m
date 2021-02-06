syms q1(t) q2(t) l1 l2  dq1 dq2 ddq1 ddq2

aaaa= (l1*sin(q1) + l2*sin(q2))^2 + (l1*cos(q1) + l2*cos(q2))^2;


% first_deriv=diff(aaaa,t);
second_deriv = diff(aaaa,t,2);
% first_deriv= subs(first_deriv,{diff(q1(t),t,t),diff(q2(t),t,t)},{'v1','v2'});
% second_deriv= subs(second_deriv,{diff(q1(t),t),diff(q1(t),t,t),diff(q2(t),t),diff(q2(t),t,t)},{'v1','a1','v2','a2'});
second_deriv= subs(second_deriv,{diff(q1(t),t), diff(q1(t),t,t), diff(q2(t),t),diff(q2(t),t,t)},{'dq1','ddq1','dq2','ddq2'});

% second_deriv= subs(second_deriv,{diff(q1(t),t),diff(q2(t),t)},{'dq1','dq2'})

% first_deriv= subs(first_deriv,diff(q2(t), t),'dq2');
% simplify(first_deriv)

simplify(second_deriv)
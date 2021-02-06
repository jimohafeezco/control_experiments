function [q,v, r] = IK2(tspan)

% dt=0.1;
% tspan=0:dt:1;
% v= zeros(2,length(tspan));
dt=0.01;
q = compute(tspan);
q1 = compute(tspan+dt);
r = task(tspan);

v= (q1-q)/dt;


end



function q= compute(tspan)

q= zeros(2,length(tspan));
    
for i = 1:length(tspan)

    r = task(tspan);
    
    L = [1.0;1.0];

    h = norm(r(:,i));

    alpha = atan2(r(2,i), r(1,i));

    q1 = alpha - acos((L(1)^2 + h^2 - L(2)^2) / (2*L(1)*h)); 
    q2 = alpha + acos((L(2)^2 + h^2 - L(1)^2) / (2*L(2)*h)); 

    q(:,i) = [q1; q2];
 
end

end



function r = task(t)

phi = 2*t;
R = 0.05; Center = [0.4; 0.4];

r = Center + [R*cos(phi); R*sin(phi)] + 0.2*[R*cos(5*phi); R*sin(5*phi)];
end
function [q,v] = IK(tspan)

dt=0.01;
% tspan=0:dt:1;
% v= zeros(2,length(tspan));

q = compute(tspan);
q1= compute(tspan+dt);

v= (q1-q)/dt;



end



function q= compute(tspan)
% q= zeros(2,length(tspan));
    
q1 = sin(tspan);
q2 = sin(tspan+1);
q3 = sin(tspan-1);
q = [q1;q2;q3];

end



function r = task(t)

phi = 2*t;
R = 0.05; Center = [0.4; 0.4];

r = Center + [R*cos(phi); R*sin(phi)] + 0.2*[R*cos(5*phi); R*sin(5*phi)];
end
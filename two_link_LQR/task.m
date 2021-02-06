function r = task(t)

phi = 2*t;
R = 0.05; Center = [0.4; 0.4];

r = Center + [R*cos(phi); R*sin(phi)] + 0.2*[R*cos(5*phi); R*sin(5*phi)];
% r = Center + [R*cos(phi); R*sin(phi)];

end
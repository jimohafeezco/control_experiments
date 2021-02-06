function drawcartpend_bw(y, l1,l2)
th = y(1);
th1= y(2);
th2= y(3);

% kinematics
% x = 3;        % cart position
% th = 3*pi/2;   % pendulum angle
x=0;
y=0;
% dimensionsye
% L = 2;  % pendulum length
M=10;
W = 1*sqrt(M/5);  % cart width
H = .1*sqrt(M/5); % cart height
% wr = .2; % wheel radius
% mr = .3*sqrt(m); % mass radius

% positions
% y = wr/2; % cart vertical position
% y = wr/2+H/2; % cart vertical position
% w1x = x-W/2;
% w1y = 0;
% w2x = x+W/2;
% w2y = 0;

px1 = x + l1*cos(th);
py1 = y + l1*sin(th);
px2 = px1 + l2*cos(th1);
py2 =py1 + l2*sin(th1);
px3 = px2 + l2*cos(th2);
py3 =py2 + l2*sin(th2);
% % plot([-10 10],[0 0],'w','LineWidth',2)
% hold on
% rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1])
% rectangle('Position',[w1x,w1y,wr,wr],'Curvature',1,'FaceColor',[1 1 1],'EdgeColor',[1 1 1])
% rectangle('Position',[w2x,w2y,wr,wr],'Curvature',1,'FaceColor',[1 1 1],'EdgeColor',[1 1 1])
plot([-4 4],[0 0],'w','LineWidth',2)

hold on
rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1])
plot([x px1],[y py1],'r','LineWidth',2)
plot([px1 px2],[py1 py2],'g','LineWidth',2)
plot([px2 px3],[py2 py3],'b','LineWidth',2)
pbaspect([1 1 1])

% rectangle('Position',[px-mr/2,py-mr/2,mr,mr],'Curvature',1,'FaceColor',[.3 0.3 1],'EdgeColor',[1 1 1])

set(gca,'YTick',[])
set(gca,'XTick',[])
xlim([-4 4]);
ylim([-4 4]);
set(gca,'Color','k','XColor','w','YColor','w')
set(gcf,'Position',[10 900 800 400])
set(gcf,'Color','k')
set(gcf,'InvertHardcopy','off')   

% box off
hold off
drawnow

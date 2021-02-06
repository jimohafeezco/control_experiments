function drawcart(x,l1, l2)
%width of mass
x0 = x(:,1);
%angle of first pend
theta = x(:,2);
% angle of second pend
theta1 = x(:,3);

% mr = .3*sqrt(m); % mass radius
W =0.5;
H=0.2;
y = 0.2;

px = x0 + l1*sin(theta);
py =  l1*cos(theta);
px2= px+l2*sin(theta1);
py2= py+l2*cos(theta1);
plot([-10 10],[0 0],'w','LineWidth',2)
hold on
rectangle('Position',[x0-W/2,y,W,H],'Curvature',.1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1])

plot([x0 px],[y py],'b','LineWidth',2)
plot([px px2],[py py2],'r','LineWidth',2)

% rectangle('Position',[px-mr/2,py-mr/2,mr,mr],'Curvature',1,'FaceColor',[.3 0.3 1],'EdgeColor',[1 1 1])

% set(gca,'YTick',[])
% set(gca,'XTick',[])
xlim([-5 5]);
ylim([-2 2.5]);
set(gca,'Color','k','XColor','w','YColor','w')
set(gcf,'Position',[10 900 800 400])
set(gcf,'Color','k')
set(gcf,'InvertHardcopy','off')   

% box off
drawnow
hold off






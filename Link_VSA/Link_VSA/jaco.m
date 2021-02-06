% function k=jaco()
% syms x1 x2 x3 x4
% 
% m1=1; m2 =1; l1 = 1; l2= 1; 
% k1=1;
% k2=1;
% g1= (m1* 0.5* l1 + m2* l1)* 10 * cos(x1)+m2 * (0.5*l2)* 10 * cos(x1+x2);
% g2 =m2 * (0.5*l2)* 10 * cos(x1+x2);
% 
% G =[g1+ k2*(x1-x3); g2+ k2*(x2-x4); k1*(x3-x1); k2*(x4-x2)];
% jacoG= jacobian(G);
% jacoG0= subs(jacoG,[x1,x2,x3,x4],[0, 0, 0 ,0]);
% 
% % clear x1 x2 x3 x4
% % 
% % syms x1 x2 x3 x4
% I1 = m1 * 0.5* l1^2;
% I2 = m2 * 0.5* l2^2;
% d11=m1* (0.5*l1)^2+ m2*(l1^2+ (0.5* l2)^2+ 2*l1*(0.5* l2)* cos(x2)+ I1+I2);
% d12=m2* ((l2*0.5)^2+ (l1* 0.5* l2* cos(x2)))+I2;
% d21=m2* ((l2*0.5)^2+ (l1* 0.5* l2* cos(x2)))+I2;
% d22=m2* (0.5* l2)^2+ I2;
% % 
% d1=1*10^-5;
% d2=1*10^-5;
% M = [d11 d12 0 0;
%     d21 d22 0 0;
%     0 0  d1  0;
%     0  0  0  d2];
% Mnut= subs(M,[x1,x2,x3,x4],[0 0 0 0]);
% anut0= -inv(Mnut)*jacoG0;
% A= [zeros(4,4) eye(4);
%     anut0       zeros(4,4)];
% 
% B= [zeros(4,1);
%     inv(Mnut)*[0 ;0 ;1 ;1]];
% Q=diag([5,50,50,50, 100,100,100,100]);
% R=1;
% % sys= ss(A,B,eye(8),0)
% k= lqr(double(A),double(B),Q, R);
% end
syms x theta
g=10;
G= [0;
    g*theta]

gg= jacobian(G,[x, theta])
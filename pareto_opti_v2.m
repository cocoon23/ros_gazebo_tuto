clear;
tic

P = [1 ,3];
P(1) = P(1)*1.2;
foc = 0.1;
x = -1.5;
camera_L = [0.35, 0];
camera_R = [-0.35, 0];
B = camera_R(1)-camera_L(1);

%P = [1.74+x, 2.85];
FOV = 80/180*pi;
P_L = P(2)*tan(FOV/2)+B;
P_R = -P(2)*tan(FOV/2)+B;
u_L = (P(1)-camera_L(1))*foc/P(2)+camera_L(1);
u_R = (P(1)-camera_R(1))*foc/P(2)+camera_R(1);
alpa = pi/2 - atan(P(2)/(P(1)-camera_L(1)));
beta = pi/2 - atan(P(2)/(P(1)-camera_R(1))); 
a = sqrt(power(P(1)-u_L,2)+power((P(2)-foc),2));
b = sqrt(power(P(1)-u_R,2)+power((P(2)-foc),2));
c = 2*B;
% lb = [max(-pi/2, -(pi/2+FOV/2)+atan2(P(2), P(1)-camera_L(1))),max(-pi/2, -(pi/2+FOV/2)+atan2(P(2), P(1)-camera_R(1)))];
% ub = [min(atan2(P(2),P(1)-camera_L(1))-(pi/2-FOV/2), pi/2), min(atan2(P(2),P(1)-camera_R(1))-(pi/2-FOV/2), pi/2)];
lb = [-pi/6, -pi/6];
ub = -lb;
x0= [0,0];
k1 = 1;
k2 = 2;
% f3 = tan(theta_L)
% 

k=0.5;
FitnessFunction = @(x)simple_mult(x(1),x(2),k);
numberOfVariables = 2;
% --------------------------------------------------------

% options = optimoptions(@gamultiobj,'PlotFcn',{@gaplotpareto,@gaplotscorediversity});
% [theta,fval] = gamultiobj(FitnessFunction,numberOfVariables,[],[],[],[],lb,ub, options);
% --------------------------------------------------------
% options = optimoptions('paretosearch','ParetoSetSize',1000000);
% x = paretosearch(FitnessFunction,numberOfVariables,[],[],[],[],lb,ub, [],options);

% 
% figure
% plot(x(:,1),x(:,2),'k*')
% xlabel('x(1)')
% ylabel('x(2)')
% hold on
% xlim([-pi/6,pi/6])
% ylim([-pi/6,pi/6])
% axis square
% hold off
% --------------------------------------------------------
% 
% 
% options = optimoptions('paretosearch','PlotFcn','psplotparetof');
% x = paretosearch(FitnessFunction,2,[],[],[],[],lb,ub,[],options);

% --------------------------------------------------------
options = optimoptions('paretosearch','PlotFcn',{'psplotparetof' 'psplotparetox'});
% rng default % For reproducibility
[x,fval] = paretosearch(FitnessFunction,2,[],[],[],[],lb,ub);

x_sort = sortrows(x,1);

toc



function f = simple_mult(theta_L, theta_R,k)
P = [1*1.2 ,3];
foc = 0.1;
x = -1.5;
camera_L = [-0.35, 0];
camera_R = [0.35, 0];
B = camera_R(1)-camera_L(1);
% P = [1.74+x, 2.85];
FOV = 90/180*pi;
P_L = P(2)*tan(FOV/2)+B;
P_R = -P(2)*tan(FOV/2)+B;
u_L = (P(1)-camera_L(1))*foc/P(2)+camera_L(1);
u_R = (P(1)-camera_R(1))*foc/P(2)+camera_R(1);
alpa = pi/2 - atan(P(2)/(P(1)-camera_L(1)));
beta = pi/2 - atan(P(2)/(P(1)-camera_R(1))); 
a = sqrt(power(P(1)-u_L,2)+power((P(2)-foc),2));
b = sqrt(power(P(1)-u_R,2)+power((P(2)-foc),2));
c = 2*B;
lb = [max(-pi/2, -(pi/2+FOV/2)+atan2(P(2), P(1)-camera_L(1))),max(-pi/2, -(pi/2+FOV/2)+atan2(P(2), P(1)-camera_R(1)))];
ub = [min(atan2(P(2),P(1)-camera_L(1))-(pi/2-FOV/2)), min(atan2(P(2),P(1)-camera_R(1))-(pi/2-FOV/2))];

f(:,1) = k*(sqrt(power(P(1)-(P(1)-camera_L(1))./(P(2)-(P(1)-camera_L(1)).*tan(theta_L)).*(camera_L(1).*P(2)/(P(1)-camera_L(1))-tan(theta_L).*(sin(theta_L)+camera_L(1))+cos(theta_L)),2) ...
    + power(P(2)-P(2)./(P(1)-camera_L(1)).*(((P(1)-camera_L(1))./(P(2)-(P(1)-camera_L(1)).*tan(theta_L)).*(camera_L(1).*P(2)./(P(1)-camera_L(1))-tan(theta_L).*(sin(theta_L)+camera_L(1))+cos(theta_L)))-camera_L(1)),2)) ...
    + sqrt(power(P(1)-((P(1)-camera_R(1))./(P(2)-(P(1)-camera_R(1)).*tan(theta_R)).*(camera_R(1)*P(2)./(P(1)-camera_R(1))-tan(theta_R).*(sin(theta_R)+camera_R(1))+cos(theta_R))),2) ...
    + power(P(2)-P(2)./(P(1)-camera_R(1)).*(((P(1)-camera_R(1))./(P(2)-(P(1)-camera_R(1)).*tan(theta_R)).*(camera_R(1).*P(2)./(P(1)-camera_R(1))-tan(theta_R).*(sin(theta_R)+camera_R(1))+cos(theta_R)))-camera_R(1)),2)));
f(:,2) = (1-k)*(abs(P(2).*tan(FOV/2-theta_L)-P(2).*tan(FOV/2-theta_R)-B)...
    + abs(-P(2).*tan(FOV/2+theta_R)+B+P(2).*tan(FOV/2+theta_L)));
end

clear all 
close all
clc
Ts=0.05;
tfin=200;
V=0.37175;
L=0.264;
width=0.6;
eL0=-0.3;
eL_lim=0.3;
delta_max=pi/6;
delta_min=-delta_max;
theta_fw_sat=pi/3;
x0=0;
y0=0;
theta0 = 0.1;


%% ALPHA POSITIVE
close all
alpha = [pi/18 pi/12 pi/6 pi/4 pi/3];
alpha_name = ["pi/18" "pi/12" "pi/6" "pi/4" "pi/3"];
for i=1:length(alpha)
    if alpha(i)==pi/6
        e=1;
    end
beta = -sign(alpha(i))*(pi-abs(alpha(i)))/2;
%waypoint
A=[0,0];
tAB=[10,0];
B=A+tAB;
tBC=[10*cos(alpha(i)),10*sin(alpha(i))];
C=B+tBC;
h=width/cos(alpha(i));
x_change_fin=(width/(2*tan(beta)))+B(1);
x_change_init=(-width/tan(alpha(i)))+(width/(2*tan(beta)))+B(1);
biscx=x_change_fin:0.0001:2*B(1)-x_change_fin;
bisec= B(2)+tan(beta)*(biscx-B(1));
BClimx1 = x_change_fin:0.01:C(1);
BClimx2 = 2*B(1)-x_change_fin:0.01:C(1);
BC_sup = h/2 + tan(alpha(i)).*(BClimx1-B(1));
BC_inf = -h/2 + tan(alpha(i)).*(BClimx2-B(1));
% load current invariant set
load("eL0_bicycle_draw.mat");
load("thetamax_fwsat.mat");
load("thetamin_fwsat.mat");
eL_max=width/2;
eL_min=-eL_max;
eL0=eL_max;
theta_max_next=acos((sin(delta_min).*(eL_max-eL0)./L)+1)-delta_min;
theta_min_next=-acos((sin(delta_max).*(eL_min-eL0)./L)+1)-delta_max;
theta_fw=intervalIntersection([theta_min_next,theta_max_next],[-theta_fw_sat,theta_fw_sat]);
theta_min_fw=theta_fw(1);
theta_max_fw=theta_fw(2);
theta_max_curr=theta_max_fw+alpha(i);
theta_min_curr=theta_min_fw+alpha(i);
theta_max_draw = theta_max_curr*ones([1 length(eL0_bicycle_draw)]);
theta_min_draw = theta_min_curr*ones([1 length(eL0_bicycle_draw)]);
eL_max_inter=eL_max-(L*(cos(theta_max_curr+delta_min)-1)/sin(delta_min));
eL_min_inter=eL_min-(L*(cos(-theta_min_curr-delta_max)-1)/sin(delta_max));
% figure
figure
% CIS S^_{i}
subplot(2,2,1)
if abs(eL_max_inter-eL_max)<=0.001
eL0=eL_min:0.001:eL_min_inter;
ymax_inter=theta_max_curr-eL0+eL0;
ymin_invset=-acos((sin(delta_max).*(eL_min-eL0)./L)+1)-delta_max;
ymin_invset=max(ymin_invset,-theta_fw_sat);
shade(eL0,ymax_inter,eL0,ymin_invset,'FillType',[1 2],'FillColor',[1 1 0])
hold on
eL0=eL_min_inter:0.001:eL_max;
ymax_inter=theta_max_curr-eL0+eL0;
ymin_inter=theta_min_curr-eL0+eL0;
shade(eL0,ymax_inter,eL0,ymin_inter,'FillType',[1 2],'FillColor',[1 1 0])
hold on
elseif abs(eL_min_inter-eL_min)<=0.001
eL0=eL_min:0.001:eL_max_inter;
ymax_inter=theta_max_curr-eL0+eL0;
ymin_inter=theta_min_curr-eL0+eL0;
shade(eL0,ymax_inter,eL0,ymin_inter,'FillType',[1 2],'FillColor',[1 1 0])
hold on
eL0=eL_max_inter:0.001:eL_max;
ymax_invset=acos((sin(delta_min).*(eL_max-eL0)./L)+1)-delta_min;
ymax_invset=min(ymax_invset,theta_fw_sat);
ymin_inter=theta_min_curr-eL0+eL0;
shade(eL0,ymax_invset,eL0,ymin_inter,'FillType',[1 2],'FillColor',[1 1 0])
hold on
else
eL0=eL_min:0.001:eL_min_inter;
ymax_inter=theta_max_curr-eL0+eL0;
ymin_invset=-acos((sin(delta_max).*(eL_min-eL0)./L)+1)-delta_max;
ymin_invset=max(ymin_invset,-theta_fw_sat);
shade(eL0,ymax_inter,eL0,ymin_invset,'FillType',[1 2],'FillColor',[1 1 0])
hold on
eL0=eL_min_inter:0.001:eL_max_inter;
ymax_inter=theta_max_curr-eL0+eL0;
ymin_inter=theta_min_curr-eL0+eL0;
shade(eL0,ymax_inter,eL0,ymin_inter,'FillType',[1 2],'FillColor',[1 1 0])
hold on
eL0=eL_max_inter:0.001:eL_max;
ymax_invset=acos((sin(delta_min).*(eL_max-eL0)./L)+1)-delta_min;
ymax_invset=min(ymax_invset,theta_fw_sat);
ymin_inter=theta_min_curr-eL0+eL0;
shade(eL0,ymax_invset,eL0,ymin_inter,'FillType',[1 2],'FillColor',[1 1 0])
hold on
end
% lane "i"
plot(eL0_bicycle_draw,thetamax_fwsat,'Color',[0.6157 0.7333 0.3804])
hold on
plot(eL0_bicycle_draw,thetamin_fwsat,'Color',[0.6157 0.7333 0.3804])
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'Color',[0.6157 0.7333 0.3804])
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'Color',[0.6157 0.7333 0.3804])
hold on
% lane "i+1" constraints over CIS lane "i"
plot(eL0_bicycle_draw,theta_max_draw,"Color",[1.0000 0.6000 0.2000])
hold on
plot(eL0_bicycle_draw,theta_min_draw,"Color",[1.0000 0.6000 0.2000])
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta_min_draw(1) theta_max_draw(1)],"Color",[1.0000 0.6000 0.2000])
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta_min_draw(end) theta_max_draw(end)],"Color",[1.0000 0.6000 0.2000])
axis padded
xlabel('e_{L} (m)');
ylabel('e_{\theta} (rad)');

title(sprintf("New section alpha positive \alpha = %s rad",alpha_name(i)))
% CIS S^_{i+1}
subplot(2,2,2)
% lane "i+1"
plot(eL0_bicycle_draw,thetamax_fwsat,'Color',[0.8431 0.8902 0.7490])
hold on
plot(eL0_bicycle_draw,thetamin_fwsat,'Color',[0.8431 0.8902 0.7490])
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'Color',[0.8431 0.8902 0.7490])
hold on
% lane "i+1" constraints over CIS lane "i+1"
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'Color',[1.0000 0.6000 0.2000])
axis padded
xlabel('e_{L} (m)');
ylabel('e_{\theta} (rad)');
%legend('\delta_{min}','\delta_{max}')
%title('New section alpha positive')
title(sprintf("New section alpha positive alpha = %s rad",alpha_name(i)))
subplot(2,2,3:4)
%tram AB
plot([A(1) B(1)],[A(2) B(2)],'LineStyle','-.','Color',[0.6157 0.7333 0.3804])
hold on
plot([A(1) B(1)],[0.3 0.3],'LineStyle','--','Color',[0.6157 0.7333 0.3804])
hold on
plot([A(1) B(1)],[-0.3 -0.3],'LineStyle','--','Color',[0.6157 0.7333 0.3804])
hold on
%tram BC
plot([B(1) C(1)],[B(2) C(2)],'LineStyle','-.','Color',[0.8431 0.8902 0.7490])
hold on
plot(BClimx1,BC_sup,'LineStyle','--','Color',[0.8431 0.8902 0.7490])
hold on
plot(BClimx2,BC_inf,'LineStyle','--','Color',[0.8431 0.8902 0.7490])
hold on
% waypoints
plot(A(1),A(2),'diamond','Color',[0.5 0.5 0.5])
hold on
plot(B(1),B(2),'diamond','Color',[0.5 0.5 0.5])
hold on
plot(C(1),C(2),'diamond','Color',[0.5 0.5 0.5])
% bisectriu
%hold on
%plot(biscx,bisec);
% set intersection
hold on
plot([x_change_init x_change_fin],[-width/2 width/2],"Color",[1.0000    0.6000    0.2000]);
axis padded
end

%% ALPHA NEGATIVE
% close all
alpha = [-pi/18 -pi/12 -pi/6 -pi/4 -pi/3];
alpha_name = ["-pi/18" "-pi/12" "-pi/6" "-pi/4" "-pi/3"];
for i=1:length(alpha)
beta = -sign(alpha(i))*(pi-abs(alpha(i)))/2;
%waypoint
A=[0,0];
tAB=[10,0];
B=A+tAB;
tBC=[10*cos(alpha(i)),10*sin(alpha(i))];
C=B+tBC;
h=width/cos(alpha(i));
x_change_fin=(-width/(2*tan(beta)))+B(1);
x_change_init=(width/tan(alpha(i)))-(width/(2*tan(beta)))+B(1);
biscx=x_change_fin:0.0001:2*B(1)-x_change_fin;
bisec= B(2)+tan(beta)*(biscx-B(1));
BClimx1 = 2*B(1)-x_change_fin:0.01:C(1);
BClimx2 = x_change_fin:0.01:C(1);
BC_sup = h/2 + tan(alpha(i)).*(BClimx1-B(1));
BC_inf = -h/2 + tan(alpha(i)).*(BClimx2-B(1));
% load current invariant set
load("eL0_bicycle_draw.mat");
load("thetamax_fwsat.mat");
load("thetamin_fwsat.mat");
eL_max=width/2;
eL_min=-eL_max;
eL0=eL_min;
theta_max_next=acos((sin(delta_min).*(eL_max-eL0)./L)+1)-delta_min;
theta_min_next=-acos((sin(delta_max).*(eL_min-eL0)./L)+1)-delta_max;
theta_max_curr=theta_max_next+alpha(i);
theta_min_curr=theta_min_next+alpha(i);
theta_max_draw = theta_max_curr*ones([1 length(eL0_bicycle_draw)]);
theta_min_draw = theta_min_curr*ones([1 length(eL0_bicycle_draw)]);
eL_max_inter=eL_max-(L*(cos(theta_max_curr+delta_min)-1)/sin(delta_min));
eL_min_inter=eL_min-(L*(cos(-theta_min_curr-delta_max)-1)/sin(delta_max));
figure
subplot(1,2,1)
eL0=eL_min:0.001:eL_min_inter;
ymax_inter=theta_max_curr-eL0+eL0;
ymin_invset=-acos((sin(delta_max).*(eL_min-eL0)./L)+1)-delta_max;
shade(eL0,ymax_inter,eL0,ymin_invset,'FillType',[1 2],'FillColor',[1 1 0])
hold on
eL0=eL_min_inter:0.001:eL_max_inter;
ymax_inter=theta_max_curr-eL0+eL0;
ymin_inter=theta_min_curr-eL0+eL0;
shade(eL0,ymax_inter,eL0,ymin_inter,'FillType',[1 2],'FillColor',[1 1 0])
hold on
eL0=eL_max_inter:0.001:eL_max;
ymax_invset=acos((sin(delta_min).*(eL_max-eL0)./L)+1)-delta_min;
ymin_inter=theta_min_curr-eL0+eL0;
shade(eL0,ymax_invset,eL0,ymin_inter,'FillType',[1 2],'FillColor',[1 1 0])
% CIS lane "i"
plot(eL0_bicycle_draw,thetamax_fwsat,'b')
hold on
plot(eL0_bicycle_draw,thetamin_fwsat,'b')
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'b')
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'b')
hold on
% CIS lane "i+1" constraints over CIS lane "i"
plot(eL0_bicycle_draw,theta_max_draw,"Color",[0.4660 0.6740 0.1880])
hold on
plot(eL0_bicycle_draw,theta_min_draw,"Color",[0.4660 0.6740 0.1880])
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta_min_draw(1) theta_max_draw(1)],"Color",[0.4660 0.6740 0.1880])
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta_min_draw(end) theta_max_draw(end)],"Color",[0.4660 0.6740 0.1880])
axis padded
xlabel('e_{L} (m)');
ylabel('e_{\theta} (rad)');
%legend('\delta_{min}','\delta_{max}')
%title('New section alpha positive')
title(sprintf("New section alpha positive alpha = %s rad",alpha_name(i)))
subplot(1,2,2)
%tram AB
plot([A(1) B(1)],[A(2) B(2)],'LineStyle','-.','Color',[0 0 0])
hold on
plot([A(1) B(1)],[0.3 0.3],'LineStyle','--','Color',[0.5 0.5 0.5])
hold on
plot([A(1) B(1)],[-0.3 -0.3],'LineStyle','--','Color',[0.5 0.5 0.5])
hold on
%tram BC
plot([B(1) C(1)],[B(2) C(2)],'LineStyle','-.','Color',[0 0 0])
hold on
plot(BClimx1,BC_sup,'LineStyle','--','Color',[0.5 0.5 0.5])
hold on
plot(BClimx2,BC_inf,'LineStyle','--','Color',[0.5 0.5 0.5])
hold on
% waypoints
plot(A(1),A(2),'diamond','Color',[0.8500 0.3250 0.0980])
hold on
plot(B(1),B(2),'diamond','Color',[0.8500 0.3250 0.0980])
hold on
plot(C(1),C(2),'diamond','Color',[0.8500 0.3250 0.0980])

hold on
plot(biscx,bisec);

hold on
plot([x_change_init x_change_fin],[width/2 -width/2],"Color",[0.4660 0.6740 0.1880]);
axis padded
end

%%
clc
N=40;
Ts=0.05;
eL_max=width/2;
eL_min=-eL_max;
x=zeros([1 N]);
y=zeros([1 N]);
theta=zeros([1 N]);
e_L=zeros([1 N]);
e_theta=zeros([1 N]);
x(1)= 0;
y(1) = -0.3;
theta(1) = acos((sin(delta_min).*(eL_max-eL_min)./L)+1)-delta_min;
alpha=pi/6;
for i=1:N
    [e_L(i),e_theta(i)]=safe_frame(x(i),y(i),theta(i),[A;B]);
    if (e_theta(i)<=pi/6*2)
        sprintf('Number of steps needed to get the intersection rectangle: %i, as e_theta = %f rad with Ts = %f s.',i,e_theta(i),Ts)
        break;
    end
    [x(i+1),y(i+1),theta(i+1)] = nextBicycleStates(x(i),y(i),theta(i),delta_min,Ts);
end
theta_max_draw = (pi/6*2)*ones([1 length(eL0_bicycle_draw)]);
theta_min_draw = ((-acos((sin(delta_max).*(eL_min-eL_max)./L)+1)-delta_max) + pi/6)*ones([1 length(eL0_bicycle_draw)]);
figure
plot(eL0_bicycle_draw,thetamax_fwsat,'b')
hold on
plot(eL0_bicycle_draw,thetamin_fwsat,'b')
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'b')
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'b')
hold on
plot(e_L(1:i),e_theta(1:i),'or')
plot(eL0_bicycle_draw,theta_max_draw,"Color",[0.4660 0.6740 0.1880])
hold on
plot(eL0_bicycle_draw,theta_min_draw,"Color",[0.4660 0.6740 0.1880])
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta_min_draw(1) theta_max_draw(1)],"Color",[0.4660 0.6740 0.1880])
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta_min_draw(end) theta_max_draw(end)],"Color",[0.4660 0.6740 0.1880])
axis padded
xlabel('e_{L} (m)');
ylabel('e_{\theta} (rad)');

%%

%%
clc
N=40;
Ts=0.05;
eL_max=width/2;
eL_min=-eL_max;
x=zeros([1 N]);
y=zeros([1 N]);
theta=zeros([1 N]);
e_L=zeros([1 N]);
e_theta=zeros([1 N]);
x(1)= 0;
y(1) = -0.3;
theta(1) = acos((sin(delta_min).*(eL_max-eL_min)./L)+1)-delta_min;
if theta(1)<=-5*pi/12
    theta(1)=-5*pi/12;
else
    theta(1)=5*pi/12;
end
alpha=pi/6;
for i=1:N
    [e_L(i),e_theta(i)]=safe_frame(x(i),y(i),theta(i),[A;B]);
    if (e_theta(i)<=pi/6*2)
        sprintf('Number of steps needed to get the intersection rectangle: %i, as e_theta = %f rad with Ts = %f s.',i,e_theta(i),Ts)
        break;
    end
    [x(i+1),y(i+1),theta(i+1)] = nextBicycleStates(x(i),y(i),theta(i),delta_min,Ts);
end
theta_max_draw = (pi/6*2)*ones([1 length(eL0_bicycle_draw)]);
theta_min_draw = ((-acos((sin(delta_max).*(eL_min-eL_max)./L)+1)-delta_max) + pi/6)*ones([1 length(eL0_bicycle_draw)]);
theta_max_safe = 5*pi/12*ones([1 length(eL0_bicycle_draw)]);
theta_min_safe = -5*pi/12*ones([1 length(eL0_bicycle_draw)]);
figure
plot(eL0_bicycle_draw,thetamax_fwsat,'b')
hold on
plot(eL0_bicycle_draw,thetamin_fwsat,'b')
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'b')
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'b')
hold on
plot(e_L(1:i),e_theta(1:i),'or')
hold on
plot(eL0_bicycle_draw,theta_max_draw,"Color",[0.4660 0.6740 0.1880])
hold on
plot(eL0_bicycle_draw,theta_min_draw,"Color",[0.4660 0.6740 0.1880])
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta_min_draw(1) theta_max_draw(1)],"Color",[0.4660 0.6740 0.1880])
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta_min_draw(end) theta_max_draw(end)],"Color",[0.4660 0.6740 0.1880])
hold on
plot(eL0_bicycle_draw,theta_max_safe,"Color",[0.8500 0.3250 0.0980])
hold on
plot(eL0_bicycle_draw,theta_min_safe,"Color",[0.8500 0.3250 0.0980])
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta_min_safe(1) theta_max_safe(1)],"Color",[0.8500 0.3250 0.0980])
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta_min_safe(end) theta_max_safe(end)],"Color",[0.8500 0.3250 0.0980])
axis padded
xlabel('e_{L} (m)');
ylabel('e_{\theta} (rad)');
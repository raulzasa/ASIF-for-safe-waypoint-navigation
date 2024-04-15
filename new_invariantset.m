close all
clear all
clc
Ts=0.05;
tfin=200;
V=1;
L=0.264;
width=0.6;
eL_min=-width/2;
eL_max=width/2;
delta_max=pi/6;
delta_min=-delta_max;
%
load("eL0_bicycle_draw.mat");
load("theta0max_bicycle_draw.mat");
load("theta0min_bicycle_draw.mat");
%
theta_fw_sat=pi/12;
%
eL0=linspace(eL_min,eL_max,1000);
theta_traj_max=zeros([1 length(eL0)]);
theta_traj_min=zeros([1 length(eL0)]);
theta_min=zeros([1 length(eL0)]);
theta_max=zeros([1 length(eL0)]);
for i=1:length(eL0)
    theta_traj_max(i)=acos((sin(delta_min).*(eL_max-eL0(i))./L)+1)-delta_min;
    theta_traj_min(i)=-acos((sin(delta_max).*(eL_min-eL0(i))./L)+1)-delta_max;
    th_int=intervalIntersection([theta_traj_min(i),theta_traj_max(i)],[-theta_fw_sat,theta_fw_sat]);
    theta_min(i)=th_int(1);
    theta_max(i)=th_int(2);
end
figure
plot(eL0,theta_traj_max,'r')
hold on
plot(eL0,theta_traj_min,'r')
hold on
plot([eL0(1) eL0(1)],[theta_traj_min(1) theta_traj_max(1)],'r')
hold on
plot([eL0(end) eL0(end)],[theta_traj_min(end) theta_traj_max(end)],'r')
hold on
plot(eL0,theta_max,'b')
hold on
plot(eL0,theta_min,'b')
hold on
plot([eL0(1) eL0(1)],[theta_min(1) theta_max(1)],'b')
hold on
plot([eL0(end) eL0(end)],[theta_min(end) theta_max(end)],'b')
axis padded
xlabel('e_{L} (m)');
ylabel('\theta (rad)');
legend({'\mathcal{S}','\mathcal{S}_{fw}'},'Location','northeast')
eL0_fwsat = eL0;
thetamax_fwsat = theta_max;
thetamin_fwsat = theta_min;
save eL0_fwsat4.mat eL0_fwsat
save thetamax_fwsat4.mat thetamax_fwsat
save thetamin_fwsat4.mat thetamin_fwsat
%% CHANGE ALPHA POSITIVE

alpha = [pi/6 pi/12 pi/18 pi/36 pi/180];
alpha_name = ["pi/6" "pi/12" "pi/18" "pi/36" "pi/180"];
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
load("eL0_fwsat.mat");
load("thetamax_fwsat.mat");
load("thetamin_fwsat.mat");
eL_max=width/2;
eL_min=-eL_max;
eL0=eL_max;
% classic invariant Set + fw saturation
theta_max_next=acos((sin(delta_min).*(eL_max-eL0)./L)+1)-delta_min;
theta_max_next=min(theta_max_next,theta_fw_sat);
theta_min_next=-acos((sin(delta_max).*(eL_min-eL0)./L)+1)-delta_max;
theta_min_next=max(theta_min_next,-theta_fw_sat);
% 
theta_max_curr=theta_max_next+alpha(i);
theta_min_curr=theta_min_next+alpha(i);

theta_max_draw = theta_max_curr*ones([1 length(eL0_fwsat)]);
theta_min_draw = theta_min_curr*ones([1 length(eL0_fwsat)]);
eL_max_inter=eL_max-(L*(cos(theta_max_curr+delta_min)-1)/sin(delta_min));
eL_min_inter=eL_min-(L*(cos(-theta_min_curr-delta_max)-1)/sin(delta_max));
figure
subplot(1,2,1)
if abs(eL_max_inter-eL_max)<=0.001
eL0=eL_min:0.001:eL_min_inter;
ymax_inter=theta_max_curr-eL0+eL0;
ymin_invset=-acos((sin(delta_max).*(eL_min-eL0)./L)+1)-delta_max;
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
ymin_inter=theta_min_curr-eL0+eL0;
shade(eL0,ymax_invset,eL0,ymin_inter,'FillType',[1 2],'FillColor',[1 1 0])
hold on
else
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
hold on
end

% CIS lane "i"
plot(eL0_fwsat,thetamax_fwsat,'b')
hold on
plot(eL0_fwsat,thetamin_fwsat,'b')
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'b')
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'b')
hold on
% CIS lane "i+1" constraints over CIS lane "i"
plot(eL0_fwsat,theta_max_draw,"Color",[0.4660 0.6740 0.1880])
hold on
plot(eL0_fwsat,theta_min_draw,"Color",[0.4660 0.6740 0.1880])
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[theta_min_draw(1) theta_max_draw(1)],"Color",[0.4660 0.6740 0.1880])
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[theta_min_draw(end) theta_max_draw(end)],"Color",[0.4660 0.6740 0.1880])
axis padded
xlabel('e_{L} (m)');
ylabel('\theta (rad)');
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
% bisectriu
hold on
plot(biscx,bisec);
% set intersection
hold on
plot([x_change_init x_change_fin],[-width/2 width/2],"Color",[0.4660 0.6740 0.1880]);
axis padded
end
%% CHANGE ALPHA POSITIVE

alpha = [pi/6 pi/12 pi/18 pi/36 pi/180];
alpha_name = ["pi/6" "pi/12" "pi/18" "pi/36" "pi/180"];
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
load("eL0_fwsat.mat");
load("thetamax_fwsat.mat");
load("thetamin_fwsat.mat");
load("eL0_bicycle_draw.mat");
load("theta0max_bicycle_draw.mat");
load("theta0min_bicycle_draw.mat");
eL_max=width/2;
eL_min=-eL_max;
eL0=eL_max;
% classic invariant Set + fw saturation
theta_max_next=acos((sin(delta_min).*(eL_max-eL0)./L)+1)-delta_min;
theta_min_next=-acos((sin(delta_max).*(eL_min-eL0)./L)+1)-delta_max;
% 
theta_max_curr=min(theta_max_next+alpha(i),theta_fw_sat);
theta_min_curr=max(theta_min_next+alpha(i),-theta_fw_sat);

theta_max_draw = (theta_max_next+alpha(i))*ones([1 length(eL0_fwsat)]);
theta_min_draw = (theta_min_next+alpha(i))*ones([1 length(eL0_fwsat)]);
eL_max_inter=eL_max-(L*(cos(theta_max_curr+delta_min)-1)/sin(delta_min));
eL_min_inter=eL_min-(L*(cos(-theta_min_curr-delta_max)-1)/sin(delta_max));
figure
subplot(1,2,1)
if abs(eL_max_inter-eL_max)<=0.001
eL0=eL_min:0.001:eL_min_inter;
ymax_inter=min(theta_max_curr,theta_fw_sat)-eL0+eL0;
ymin_invset=max(-acos((sin(delta_max).*(eL_min-eL0)./L)+1)-delta_max,-theta_fw_sat)-eL0+eL0;
shade(eL0,ymax_inter,eL0,ymin_invset,'FillType',[1 2],'FillColor',[1 1 0])
hold on
eL0=eL_min_inter:0.001:eL_max;
ymax_inter=min(theta_max_curr,theta_fw_sat)-eL0+eL0;
ymin_inter=max(theta_min_curr,-theta_fw_sat)-eL0+eL0;
shade(eL0,ymax_inter,eL0,ymin_inter,'FillType',[1 2],'FillColor',[1 1 0])
hold on
elseif abs(eL_min_inter-eL_min)<=0.001
eL0=eL_min:0.001:eL_max_inter;
ymax_inter=min(theta_max_curr,theta_fw_sat)-eL0+eL0;
ymin_inter=max(theta_min_curr,-theta_fw_sat)-eL0+eL0;
shade(eL0,ymax_inter,eL0,ymin_inter,'FillType',[1 2],'FillColor',[1 1 0])
hold on
eL0=eL_max_inter:0.001:eL_max;
ymax_invset=min(acos((sin(delta_min).*(eL_max-eL0)./L)+1)-delta_min,theta_fw_sat);
ymin_inter=max(theta_min_curr,-theta_fw_sat)-eL0+eL0;
shade(eL0,ymax_invset,eL0,ymin_inter,'FillType',[1 2],'FillColor',[1 1 0])
hold on
else
eL0=eL_min:0.001:eL_min_inter;
ymax_inter=min(theta_max_curr,theta_fw_sat)-eL0+eL0;
ymin_invset=max(-acos((sin(delta_max).*(eL_min-eL0)./L)+1)-delta_max,-theta_fw_sat);
shade(eL0,ymax_inter,eL0,ymin_invset,'FillType',[1 2],'FillColor',[1 1 0])
hold on
eL0=eL_min_inter:0.001:eL_max_inter;
ymax_inter=min(theta_max_curr,theta_fw_sat)-eL0+eL0;
ymin_inter=max(theta_min_curr,-theta_fw_sat)-eL0+eL0;
shade(eL0,ymax_inter,eL0,ymin_inter,'FillType',[1 2],'FillColor',[1 1 0])
hold on
eL0=eL_max_inter:0.001:eL_max;
ymax_invset=min(acos((sin(delta_min).*(eL_max-eL0)./L)+1)-delta_min,theta_fw_sat);
ymin_inter=max(theta_min_curr,-theta_fw_sat)-eL0+eL0;
shade(eL0,ymax_invset,eL0,ymin_inter,'FillType',[1 2],'FillColor',[1 1 0])
hold on
end
% CIS tal cual
plot(eL0_bicycle_draw,theta0max_bicycle_draw,'r')
hold on
plot(eL0_bicycle_draw,theta0min_bicycle_draw,'r')
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta0min_bicycle_draw(1) theta0max_bicycle_draw(1)],'r')
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta0min_bicycle_draw(end) theta0max_bicycle_draw(end)],'r')
hold on

% CIS lane "i"
plot(eL0_fwsat,thetamax_fwsat,'b')
hold on
plot(eL0_fwsat,thetamin_fwsat,'b')
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'b')
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'b')
hold on
% CIS lane "i+1" constraints over CIS lane "i"
plot(eL0_fwsat,theta_max_draw,"Color",[0.4660 0.6740 0.1880])
hold on
plot(eL0_fwsat,theta_min_draw,"Color",[0.4660 0.6740 0.1880])
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[theta_min_draw(1) theta_max_draw(1)],"Color",[0.4660 0.6740 0.1880])
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[theta_min_draw(end) theta_max_draw(end)],"Color",[0.4660 0.6740 0.1880])
axis padded
xlabel('e_{L} (m)');
ylabel('\theta (rad)');

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
% bisectriu
hold on
plot(biscx,bisec);
% set intersection
hold on
plot([x_change_init x_change_fin],[-width/2 width/2],"Color",[0.4660 0.6740 0.1880]);
axis padded
end

%% ESTUDI DE L'EVOLUCIÓ DE L'ERROR D'ORIENTACIÓ

delta_range = linspace(delta_min,delta_max,111);
etheta_0=pi/3;
etheta_t=[];
for i=1:length(delta_range)
    etheta_t(i)=(V*Ts/L)*sin(delta_range(i));
end
figure
plot(delta_range,etheta_t)

%% INVARIANT SET FOR CHANGING LANE: ALPHA = PI/6
close all
clear all
clc
Ts=0.05;
tfin=200;
V=0.37175;
L=0.264;
width=0.6;
eL_min=-width/2;
eL_max=width/2;
delta_max=pi/6;
delta_min=-delta_max;
% Orientation saturation according lane motion
theta_fw_sat=pi/3;
% Difference between lane orientations
alpha=pi/6;
% lateral error vector
load("eL0_bicycle_draw.mat");
% new orientations error bounds
theta_fw_alpha=zeros([length(eL0_bicycle_draw) 2]);
for i=1:length(eL0_bicycle_draw)
    theta_fw_alpha(i,:)=theta_bounds_fw_alpha(eL0_bicycle_draw(i),theta_fw_sat,alpha);
end
figure
plot(eL0_bicycle_draw,theta_fw_alpha(:,1),'r')
hold on
plot(eL0_bicycle_draw,theta_fw_alpha(:,2),'r')
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta_fw_alpha(1,1) theta_fw_alpha(1,2)],'r')
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta_fw_alpha(end,1) theta_fw_alpha(end,2)],'r')
hold on


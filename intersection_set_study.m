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
eL0_next = linspace(eL_min,eL_max,11);
% new orientations error bounds
theta_fw_alpha=zeros([length(eL0_bicycle_draw) 2]);
figure
for j=1:length(eL0_next)
for i=1:length(eL0_bicycle_draw)
    theta_fw_alpha(i,:)=etheta_bounds_fw_alpha(eL0_bicycle_draw(i),theta_fw_sat,alpha,eL0_next(j));
end

plot(eL0_bicycle_draw,theta_fw_alpha(:,1))
hold on
plot(eL0_bicycle_draw,theta_fw_alpha(:,2))
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta_fw_alpha(1,1) theta_fw_alpha(1,2)])
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta_fw_alpha(end,1) theta_fw_alpha(end,2)])
hold on

end
%%
alpha=pi/18;
theta_fw_sat=pi/12;
load("eL0_bicycle_draw.mat");
for i=1:length(eL0_bicycle_draw)
    theta_fw_alpha(i,:)=etheta_bounds_fw_alpha(eL0_bicycle_draw(i),theta_fw_sat,alpha,eL_max);
end
%save theta_fw_alpha_draw theta_fw_alpha
figure
% S
load("theta0max_bicycle_draw.mat")
load("theta0min_bicycle_draw.mat")
plot(eL0_bicycle_draw,theta0min_bicycle_draw,'g')
hold on
plot(eL0_bicycle_draw,theta0max_bicycle_draw,'g')
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta0min_bicycle_draw(1) theta0max_bicycle_draw(1)],'g')
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta0min_bicycle_draw(end) theta0max_bicycle_draw(end)],'g')
hold on
% Sfw
load("eL0_fwsat4.mat")
load("thetamax_fwsat4.mat")
load("thetamin_fwsat4.mat")
plot(eL0_fwsat,thetamin_fwsat,'b')
hold on
plot(eL0_fwsat,thetamax_fwsat,'b')
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'b')
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'b')
hold on
% Ci
plot(eL0_bicycle_draw,theta_fw_alpha(:,1),'r')
hold on
plot(eL0_bicycle_draw,theta_fw_alpha(:,2),'r')
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta_fw_alpha(1,1) theta_fw_alpha(1,2)],'r')
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta_fw_alpha(end,1) theta_fw_alpha(end,2)],'r')
hold off

%% RESTRINGINT DELTA PER MANTENIR INVARIANT iCi+1
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
% Initial state vector
delta_range=linspace(delta_min,delta_max,41);
eL0_vec = linspace(eL_min,eL_max,11);
theta_fwa_vec=zeros([length(eL0_vec) 2]);
theta0_vec=zeros([length(eL0_vec) 11]);
for i=1:length(eL0_vec)
    theta_fwa_vec(i,:)=etheta_bounds_fw_alpha(eL0_vec(i),theta_fw_sat,alpha,eL_max);
    theta0_vec(i,:)=linspace(theta_fwa_vec(i,1),theta_fwa_vec(i,2),11);
end
% Map
beta = -sign(alpha)*(pi-abs(alpha))/2;
A=[0,0];
tAB=[10,0];
B=A+tAB;
tBC=[10*cos(alpha),10*sin(alpha)];
C=B+tBC;
h=width/cos(alpha);
x_change_fin=(width/(2*tan(beta)))+B(1);
x_change_init=(-width/tan(alpha))+(width/(2*tan(beta)))+B(1);
biscx=x_change_fin:0.0001:2*B(1)-x_change_fin;
bisec= B(2)+tan(beta)*(biscx-B(1));
BClimx1 = x_change_fin:0.01:C(1);
BClimx2 = 2*B(1)-x_change_fin:0.01:C(1);
BC_sup = h/2 + tan(alpha).*(BClimx1-B(1));
BC_inf = -h/2 + tan(alpha).*(BClimx2-B(1));
% world space
dgrid_vec=zeros([length(eL0_vec) length(theta0_vec) 2]);
x_vec=zeros([length(eL0_vec) length(theta0_vec)]);
y_vec=zeros([length(eL0_vec) length(theta0_vec)]);
theta_vec=zeros([length(eL0_vec) length(theta0_vec)]);
%Si space
eL_S1=zeros([length(eL0_vec) length(theta0_vec)]);
etheta_S1=zeros([length(eL0_vec) length(theta0_vec)]);
%Si+1 space
eL_S2=zeros([length(eL0_vec) length(theta0_vec)]);
etheta_S2=zeros([length(eL0_vec) length(theta0_vec)]);
% seed
rng(6,'twister')
for i=1:length(eL0_vec)
    y0=eL0_vec(i);
    x0=(eL0_vec(i)-(-width/2))/tan(alpha)+x_change_init;
    for j=1:size(theta0_vec,2)
        if (i==10) && (j==11)
            break;
            a=9;
        end
        theta0=theta0_vec(i,j);
        delta_grid=zeros([1,length(delta_range)]);
        for k=1:length(delta_range)
            [x_Ts,y_Ts,theta_Ts] = nextBicycleStates(x0,y0,theta0,delta_range(k),Ts);
            [eL_Ts,theta_Ts] = safe_frame(x_Ts,y_Ts,theta_Ts,[A;B]);% QUEDA DEFINIR TRAM
            p_local = transformacionHomogeneaInversa([x_Ts,y_Ts,theta_Ts],A,B);
            delta_grid(k) = isInvariantSet3(eL_Ts,theta_Ts,alpha); %EVALUAR DELTA PARA EL CURRTRAM
        end
        jmax=find(delta_grid,1,'last');
        jmin=find(delta_grid,1,'first');
        dgrid=[delta_range(jmin),delta_range(jmax)];
        delta_i = dgrid(1) + (dgrid(2)-dgrid(1))*rand();
        dgrid_vec(i,j,:)=dgrid;
        
        [x_vec(i,j),y_vec(i,j),theta_vec(i,j)] = nextBicycleStates(x0,y0,theta0,delta_i,Ts);% RANDOM DELTA
        [eL_S1(i,j),etheta_S1(i,j)] = safe_frame(x_vec(i,j),y_vec(i,j),theta_vec(i,j),[A;B]); % EVOLUTION IN Si / Ci
        [eL_S2(i,j),etheta_S2(i,j)] = safe_frame(x_vec(i,j),y_vec(i,j),theta_vec(i,j),[B;C]); % EVOLUTION IN Si+1
        % [eL_S2(i,j),etheta_S2(i,j)] = safe_frame(x_vec(i,j),y_vec(i,j),theta_vec(i,j),[B;C]); % EVOLUTION IN Si+1
        % isInvariantSet3(eL_Tsnext,theta_Tsnext,alpha);
    end
end
%%
load("eL0_fwsat.mat")
load("thetamax_fwsat.mat")
load("thetamin_fwsat.mat")
load("eL0_bicycle_draw.mat");
load("theta_fw_alpha_draw.mat");
figure
% Sfw i
plot(eL0_fwsat,thetamin_fwsat,'b')
hold on
plot(eL0_fwsat,thetamax_fwsat,'b')
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'b')
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'b')
hold on
% C i
plot(eL0_bicycle_draw,theta_fw_alpha(:,1),'r')
hold on
plot(eL0_bicycle_draw,theta_fw_alpha(:,2),'r')
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta_fw_alpha(1,1) theta_fw_alpha(1,2)],'r')
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta_fw_alpha(end,1) theta_fw_alpha(end,2)],'r')
hold on
for i=1:length(eL0_vec)
plot(eL0_vec(i)*ones([1 size(theta0_vec,2)]),theta0_vec(i,:),'o')
hold on
plot(eL_S1(i,1:j),etheta_S1(i,1:j),'*')
hold on
end
hold off
xlabel('e_{L} (m)');
ylabel('\theta (rad)');
axis padded

figure
% Sfw i+1
plot(eL0_fwsat,thetamin_fwsat,'b')
hold on
plot(eL0_fwsat,thetamax_fwsat,'b')
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'b')
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'b')
hold on
% C i+1
plot(eL0_bicycle_draw,theta_fw_alpha(:,1),'r')
hold on
plot(eL0_bicycle_draw,theta_fw_alpha(:,2),'r')
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta_fw_alpha(1,1) theta_fw_alpha(1,2)],'r')
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta_fw_alpha(end,1) theta_fw_alpha(end,2)],'r')
hold on
plot(eL_max*ones([1 size(theta0_vec,2)]),theta0_vec(i,:),'o')
hold on
for i=1:length(eL0_vec)
    plot(eL_S2(i,1:j),etheta_S2(i,1:j),'*')
    hold on
end
xlabel('e_{L} (m)');
ylabel('\theta (rad)');
axis padded

figure
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
%
for i=1:length(eL0_vec)
    y0=eL0_vec(i);
    x0=(eL0_vec(i)-(-width/2))/tan(alpha)+x_change_init;
    plot(x0,y0,'ob')
end
% next iteration
hold on
plot(x_vec(1:i,1:j),y_vec(1:i,1:j),'*')
axis padded

%% RESTRINGINT DELTA PER MANTENIR INVARIANT Si+1
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
% Vectors
delta_range=linspace(delta_min,delta_max,41);
eL0_vec = linspace(eL_min,eL_max,11);
theta_fwa_vec=zeros([length(eL0_vec) 2]);
theta0_vec=zeros([length(eL0_vec) 11]);
for i=1:length(eL0_vec)
    theta_fwa_vec(i,:)=etheta_bounds_fw_alpha(eL0_vec(i),theta_fw_sat,alpha,eL_max);
    theta0_vec(i,:)=linspace(theta_fwa_vec(i,1),theta_fwa_vec(i,2),11);
end
beta = -sign(alpha)*(pi-abs(alpha))/2;
A=[0,0];
tAB=[10,0];
B=A+tAB;
tBC=[10*cos(alpha),10*sin(alpha)];
C=B+tBC;
h=width/cos(alpha);
x_change_fin=(width/(2*tan(beta)))+B(1);
x_change_init=(-width/tan(alpha))+(width/(2*tan(beta)))+B(1);
biscx=x_change_fin:0.0001:2*B(1)-x_change_fin;
bisec= B(2)+tan(beta)*(biscx-B(1));
BClimx1 = x_change_fin:0.01:C(1);
BClimx2 = 2*B(1)-x_change_fin:0.01:C(1);
BC_sup = h/2 + tan(alpha).*(BClimx1-B(1));
BC_inf = -h/2 + tan(alpha).*(BClimx2-B(1));
% world
dgrid_vec=zeros([length(eL0_vec) length(theta0_vec) 2]);
x_vec=zeros([length(eL0_vec) length(theta0_vec)]);
y_vec=zeros([length(eL0_vec) length(theta0_vec)]);
theta_vec=zeros([length(eL0_vec) length(theta0_vec)]);
%Si
eL_S1=zeros([length(eL0_vec) length(theta0_vec)]);
etheta_S1=zeros([length(eL0_vec) length(theta0_vec)]);
%Si+1
eL_S2=zeros([length(eL0_vec) length(theta0_vec)]);
etheta_S2=zeros([length(eL0_vec) length(theta0_vec)]);
% seed
% rng(6,'twister')
% for i=1:length(eL0_vec)
%     y0=eL0_vec(i);
%     x0=(eL0_vec(i)-(-width/2))/tan(alpha)+x_change_init;
%     for j=1:size(theta0_vec,2)
        % if (i==10) && (j==11)
        %     break;
        %     a=9;
        % end
        % theta0=theta0_vec(i,j);
        % delta_grid=zeros([1,length(delta_range)]);
        % for k=1:length(delta_range)
            % [x_Ts,y_Ts,theta_Ts] = nextBicycleStates(x0,y0,theta0,delta_range(k),Ts);
            % [eL_Ts,etheta_Ts] = safe_frame(x_Ts,y_Ts,theta_Ts,[B;C]);% QUEDA DEFINIR TRAM
            % [eL_Ts2,etheta_Ts2] = e_frame([x_Ts,y_Ts,theta_Ts],B,C);
            % delta_grid(k) = isInvariantSet3(eL_Ts,etheta_Ts,alpha); %EVALUAR DELTA PARA EL CURRTRAM
            % delta_grid(k) = isInvariantSet3(eL_Ts2,etheta_Ts2,alpha); %EVALUAR DELTA PARA EL CURRTRAM
        % end
        % jmax=find(delta_grid,1,'last');
        % jmin=find(delta_grid,1,'first');
        % dgrid=[delta_range(jmin),delta_range(jmax)];
        % delta_i = dgrid(1) + (dgrid(2)-dgrid(1))*rand();
        % dgrid_vec(i,j,:)=dgrid;
        
%         [x_vec(i,j),y_vec(i,j),theta_vec(i,j)] = nextBicycleStates(x0,y0,theta0,delta_i,Ts);% RANDOM DELTA
%         [eL_S1(i,j),etheta_S1(i,j)] = e_frame([x0,y0,theta0],A,B); % EVOLUTION IN Si / Ci
%         isInvariantSet3(eL_Tsnext,theta_Tsnext,alpha);
%         [eL_S2(i,j),etheta_S2(i,j)] = e_frame([x0,y0,theta0],B,C); % EVOLUTION IN Si+1
%         isInvariantSet3(eL_Tsnext,theta_Tsnext,alpha);
%     end
% end

th_fwmax_intersect=intervalIntersection([-pi/3,pi/3],[-pi/3+alpha,pi/3+alpha]);
theta_intersect=zeros([length(eL0_vec) 2]);
for i=1:length(eL0_vec)
    theta_intersect(i,:)=intervalIntersection(th_fwmax_intersect,theta_fwa_vec(i,:));
end

% theta0_vec=linspace(th_max_intersect(1),th_max_intersect(2),11);
for i=1:length(eL0_vec)
    y0=eL0_vec(i);
    x0=(eL0_vec(i)-(-width/2))/tan(alpha)+x_change_init;
    for j=1:length(theta0_vec)
        theta0=theta0_vec(j);
        delta_grid=zeros([1,length(delta_range)]);
        [eL_S1(i,j),etheta_S1(i,j)] = safe_frame(x0,y0,theta0,[A;B]); % EVOLUTION IN Si / Ci
        AB_safe=isInvariantSet3(eL_S1(i,j),etheta_S1(i,j),alpha);
        [eL_S2(i,j),etheta_S2(i,j)] = safe_frame(x0,y0,theta0,[B;C]); % EVOLUTION IN Si+1
        BC_safe=isInvariantSet3(eL_S2(i,j),etheta_S2(i,j),alpha);
        % if (AB_safe==1) && (BC_safe==1)
        %     x_vec(i,j)=1;
        %     y_vec(i,j)=1;
        %     theta_vec(i,j)=1;
        % end
        if (BC_safe==1)
            x_vec(i,j)=1;
            y_vec(i,j)=1;
            theta_vec(i,j)=1;
        end
    end
end


%%
load("eL0_fwsat.mat");
load("thetamax_fwsat.mat");
load("thetamin_fwsat.mat");
load("eL0_bicycle_draw.mat");
load("theta_fw_alpha_draw.mat");
figure
% Sfw i
plot(eL0_fwsat,thetamin_fwsat,'b')
hold on
plot(eL0_fwsat,thetamax_fwsat,'b')
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'b')
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'b')
hold on
% C i
plot(eL0_bicycle_draw,theta_fw_alpha(:,1),'r')
hold on
plot(eL0_bicycle_draw,theta_fw_alpha(:,2),'r')
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta_fw_alpha(1,1) theta_fw_alpha(1,2)],'r')
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta_fw_alpha(end,1) theta_fw_alpha(end,2)],'r')
hold on
for i=1:length(eL0_vec)
plot(eL0_vec(i)*ones([1 length(theta0_vec)]),theta0_vec(:),'o')
hold on
plot(eL_S1(i,:),etheta_S1(i,:),'*')
hold on
end
hold off
xlabel('e_{L} (m)');
ylabel('\theta (rad)');
axis padded

figure
% Sfw i+1
plot(eL0_fwsat,thetamin_fwsat,'b')
hold on
plot(eL0_fwsat,thetamax_fwsat,'b')
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'b')
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'b')
hold on
% C i+1
plot(eL0_bicycle_draw,theta_fw_alpha(:,1),'r')
hold on
plot(eL0_bicycle_draw,theta_fw_alpha(:,2),'r')
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta_fw_alpha(1,1) theta_fw_alpha(1,2)],'r')
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta_fw_alpha(end,1) theta_fw_alpha(end,2)],'r')
hold on
plot(eL_max*ones([1 length(theta0_vec)]),theta0_vec(:),'o')
hold on
for i=1:length(eL0_vec)
    plot(eL_S2(i,:),etheta_S2(i,:),'*')
    hold on
end
xlabel('e_{L} (m)');
ylabel('\theta (rad)');
axis padded

figure
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
%
for i=1:length(eL0_vec)
    y0=eL0_vec(i);
    x0=(eL0_vec(i)-(-width/2))/tan(alpha)+x_change_init;
    plot(x0,y0,'ob')
end
% next iteration
hold on
plot(x_vec(1:i,1:j),y_vec(1:i,1:j),'*')
axis padded

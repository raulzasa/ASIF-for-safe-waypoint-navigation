%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%            Carregar circuit dodecaedre (30º)                   %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all 
close all
clc
Ts=0.1;
% tfin=0.05;
%V=0.37175;
V=1;
L=0.264;
width=0.6;
eL0=-0.3;
eL_lim=0.3;
delta_max=pi/6;
delta_min=-delta_max;
delta_vec=[delta_min,delta_max];
delta_range=linspace(delta_min,delta_max,41);
theta_fwsat=pi/3;

%delta=-pi/6;
R = 10;
N = 12;
%waypoints
wp_vec= calcularVerticesPoligonoRegular(R,[0,0],N);
wp_vec=[wp_vec(7:end,:);wp_vec(1:6,:)];
wp_vec_ext= calcularVerticesPoligonoRegular(R+((width/2)/sin(75*pi/180)),[0,0],N);
wp_vec_ext=[wp_vec_ext(7:end,:);wp_vec_ext(1:6,:)];
wp_vec_int= calcularVerticesPoligonoRegular(R-((width/2)/sin(75*pi/180)),[0,0],N);
wp_vec_int=[wp_vec_int(7:end,:);wp_vec_int(1:6,:)];
wp_frames=wp_vec(1:N,:);
%initial conditions
alpha = pi/6;
% x0= 0;
% y0= min(wp_vec(:,2));
% theta0 = 0;
wp_vec = [wp_vec;wp_vec(1,:)];
wp_vec_ext = [wp_vec_ext;wp_vec_ext(1,:)];
wp_vec_int = [wp_vec_int;wp_vec_int(1,:)];
x0 = zeros([1 N]);
y0 = zeros([1 N]);
theta0 = zeros([1 N]);
for i=1:size(wp_vec,1)-1
    % x0(i) = (wp_vec(i,1)+wp_vec(i+1,1))/2;
    % y0(i) = (wp_vec(i,2)+wp_vec(i+1,2))/2;
    % theta0(i) = atan2(wp_vec(i+1,2)-wp_vec(i,2),wp_vec(i+1,1)-wp_vec(i,1));
    x0(i) = wp_vec(i,1);
    y0(i) = wp_vec(i,2);
    theta0(i) = atan2(wp_vec(i+1,2)-wp_vec(i,2),wp_vec(i+1,1)-wp_vec(i,1));
end

a=zeros([N N]);
elong=zeros([1 N]);
eL=zeros([1 N]);
etheta=zeros([1 N]);

figure
plot(wp_vec(:,1),wp_vec(:,2),'LineStyle','-.','Color',[0 0 0],'HandleVisibility','off');
hold on
plot(wp_vec_int(:,1),wp_vec_int(:,2),'LineStyle','--','Color',[0.5 0.5 0.5],'HandleVisibility','off');
hold on
plot(wp_vec_ext(:,1),wp_vec_ext(:,2),'LineStyle','--','Color',[0.5 0.5 0.5],'HandleVisibility','off');
hold on
for j=1:N
    for i=1:N
        currwp=wp_frames(i,:);
        if i==N
            nextwp=wp_frames(1,:);
        else
            nextwp=wp_frames(i+1,:);
        end
        % [elong(i),eL(i),etheta(i)]=transformacionHomogeneaInversa([x0,y0,theta0],currwp,nextwp);
        estate=transformacionHomogeneaInversa([x0(j),y0(j),theta0(j)],currwp,nextwp);
        elong(i)=estate(1);
        eL(i)=estate(2);
        etheta(i)=estate(3);
        if isInvariantSet_fwsat(eL(i),etheta(i),theta_fwsat)
            a(j,i)=1;
            % plot(x0(j),y0(j),'*','Color',color_selector(j))
            hold on
        end
    end
    [dx,dy]=frame_maker(wp_vec(j,:),wp_vec(j+1,:));
    % Dibujar el eje X
    % quiver(x0(j),y0(j),dx(1), dx(2), 'Color',color_selector(j), 'LineWidth', 1.5);
    % hold on; 
    % % Dibujar el eje Y
    % quiver(x0(j),y0(j),dy(1), dy(2), 'Color',color_selector(j), 'LineWidth', 1.5);
    % hold on
end
eL0_localfront = linspace(-width/2,width/2,7);
etheta_localfront=zeros([2 length(eL0_localfront)]);
x_frontier=zeros([N length(eL0_localfront)]);
y_frontier=zeros([N length(eL0_localfront)]);
theta_frontier=zeros([N length(eL0_localfront) 2]);
for i=1:N
    % Frontera de intersección
    for j=1:length(eL0_localfront)
        if i==N
            p_supint=transformacionHomogeneaInversa([wp_vec_int(i+1,1),wp_vec_int(i+1,2),theta0(1)],wp_vec(i,:),wp_vec(i+1,:));
        else
            p_supint=transformacionHomogeneaInversa([wp_vec_int(i+1,1),wp_vec_int(i+1,2),theta0(i+1)],wp_vec(i,:),wp_vec(i+1,:));
        end
        x_inf_local=p_supint(1)-(p_supint(2)-(eL0_localfront(j)))/tan(alpha);
        etheta_localfront(:,j)=etheta_bounds_fw_alpha(eL0_localfront(j),theta_fwsat,alpha,width/2);
        for k=1:2
            glob_infint=transformacionHomogeneaDirecta([x_inf_local,eL0_localfront(j),etheta_localfront(k,j)],wp_vec(i,:),wp_vec(i+1,:));
            x_frontier(i,j)=glob_infint(1);
            y_frontier(i,j)=glob_infint(2);
            theta_frontier(i,j,k)=glob_infint(3);
        end
    end
    % plot([x_frontier(i,1) wp_vec_int(i+1,1)],[y_frontier(i,1) wp_vec_int(i+1,2)],'LineStyle',':','Color',[0.7 0.7 0.7])
    % hold on  
end

%% recorregut sencer 3
 
clc
tfin=100;
t=0:Ts:tfin;



x=zeros([1 length(t)]);
y=zeros([1 length(t)]);
theta=zeros([1 length(t)]);
x(1)=-2;
y(1)=-9.65926;
theta(1)=-1;
lane_label = zeros([1 length(t)]);
e_Lcurr=zeros([1 length(t)]);
e_thetacurr=zeros([1 length(t)]);
e_Lnext=zeros([1 length(t)]);
e_thetanext=zeros([1 length(t)]);
% delta=zeros(N,length(eL0_localfront),2);
dgrid_vec=zeros(1,length(delta_range));
delta_grid_vec=zeros([length(t) 2]);
delta=zeros([1 length(t)]);
delta_drunk=zeros([1 length(t)]);
% loop
lane_id=1;
lane_label = zeros([1 length(t)]);
% seed
rng(6,'twister')
for i = 1:length(t)
    lane_label(i)=lane_id;
    % Determine the next waypoint indices in a circular manner
    wp_next_index = mod(lane_id, 12) + 1;
    wp_next_next_index = mod(lane_id + 1, 12) + 1; % This will automatically wrap to 1 if lane_id + 1 > 12

    % eL & etheta w.r.t current lane
    P_localcurr = transformacionHomogeneaInversa([x(i), y(i), theta(i)], wp_vec(lane_id, :), wp_vec(wp_next_index, :));

    % eL & etheta w.r.t next lane, adjusted with mod for circular indexing
    P_localnext = transformacionHomogeneaInversa([x(i), y(i), theta(i)], wp_vec(wp_next_index, :), wp_vec(wp_next_next_index, :));

    % If C next
    if isInvariantSet_fwsat_C(P_localnext(2), P_localnext(3), theta_fwsat, alpha)
        % Your existing logic here for handling the invariant set condition
        for p=1:length(delta_range)
            [x_Ts,y_Ts,theta_Ts] = nextBicycleStates(x(i),y(i),theta(i),delta_range(p),Ts);
            P_local1 = transformacionHomogeneaInversa([x_Ts,y_Ts,theta_Ts], wp_vec(wp_next_index,:), wp_vec(wp_next_next_index,:));
            dgrid_vec(p) = isInvariantSet_fwsat_C(P_local1(2),P_local1(3),theta_fwsat,alpha);
        end
        jmax=find(dgrid_vec,1,'last');
        jmin=find(dgrid_vec,1,'first');
        dgrid=[delta_range(jmin),delta_range(jmax)];
        % Update for next iteration
        if lane_id <= 12
            lane_id = lane_id + 1; % Increment lane_id
        end

        if lane_id > 12
            lane_id = 1; % Reset lane_id to start over if it exceeds 12
        end
    else
        if isInvariantSet_fwsat(P_localnext(2), P_localnext(3), theta_fwsat)
            % Your existing logic here for handling the Sfw next condition
            for p=1:length(delta_range)
                [x_Ts,y_Ts,theta_Ts] = nextBicycleStates(x(i),y(i),theta(i),delta_range(p),Ts);
                P_local2 = transformacionHomogeneaInversa([x_Ts,y_Ts,theta_Ts], wp_vec(wp_next_index,:), wp_vec(wp_next_next_index,:));
                dgrid_vec(p) = isInvariantSet_fwsat(P_local2(2),P_local2(3),theta_fwsat);
            end
            jmax=find(dgrid_vec,1,'last');
            jmin=find(dgrid_vec,1,'first');
            dgrid=[delta_range(jmin),delta_range(jmax)];
            % Update for next iteration
            if lane_id <= 12
                lane_id = lane_id + 1; % Increment lane_id
            end

            if lane_id > 12
                lane_id = 1; % Reset lane_id to start over if it exceeds 12
            end
        else
            % If C curr
            if isInvariantSet_fwsat_C(P_localcurr(2), P_localcurr(3), theta_fwsat, alpha)
                % Your existing logic here for handling the C curr condition
                for p=1:length(delta_range)
                    [x_Ts,y_Ts,theta_Ts] = nextBicycleStates(x(i),y(i),theta(i),delta_range(p),Ts);
                    P_local3 = transformacionHomogeneaInversa([x_Ts,y_Ts,theta_Ts], wp_vec(lane_id, :), wp_vec(wp_next_index, :));
                    dgrid_vec(p) = isInvariantSet_fwsat_C(P_local3(2),P_local3(3),theta_fwsat,alpha);
                end
                jmax=find(dgrid_vec,1,'last');
                jmin=find(dgrid_vec,1,'first');
                dgrid=[delta_range(jmin),delta_range(jmax)];
            else
                if isInvariantSet_fwsat(P_localcurr(2), P_localcurr(3), theta_fwsat)
                    % Your existing logic here for handling the Sfw curr condition
                    for p=1:length(delta_range)
                        [x_Ts,y_Ts,theta_Ts] = nextBicycleStates(x(i),y(i),theta(i),delta_range(p),Ts);
                        P_local3 = transformacionHomogeneaInversa([x_Ts,y_Ts,theta_Ts], wp_vec(lane_id, :), wp_vec(wp_next_index, :));
                        dgrid_vec(p) = isInvariantSet_fwsat(P_local3(2),P_local3(3),theta_fwsat);
                    end
                    jmax=find(dgrid_vec,1,'last');
                    jmin=find(dgrid_vec,1,'first');
                    dgrid=[delta_range(jmin),delta_range(jmax)];
                    theta_C=etheta_bounds_fw_alpha(P_localcurr(2),theta_fwsat,alpha,width/2);
                    % if theta > theta_C_max
                    if theta_C(2)<=P_localcurr(3)
                        % delta_int = (0,pi/6];
                        delint=[delta_min,0];
                    end
                    % if theta < theta_C_min
                    if P_localcurr(3)<=theta_C(1)
                        % delta_int = [-pi/6,0);
                        delint=[0,delta_max];
                    end
                    if ~isempty(intervalIntersection(dgrid,delint))
                        dgrid=intervalIntersection(dgrid,delint);
                    end
                else
                    sprintf("Break at t= %d s in lane %d because eL = %f & etheta = %f", i, lane_id, P_localcurr(2), P_localcurr(3));
                    break;
                end
            end
        end
    end    
    % Your code to update delta(i) and perform nextBicycleStates calculation here
    delta_drunk(i) = delta_min + (delta_max-delta_min)*rand();
    % delta(i) = dgrid(1) + (dgrid(2)-dgrid(1))*rand();
    delta(i) = max(dgrid(1), min(dgrid(2), delta_drunk(i)));
    delta_grid_vec(i,:)=dgrid;
    [x(i+1),y(i+1),theta(i+1)] = nextBicycleStates(x(i),y(i),theta(i),delta(i),Ts);
    P_localcurr = transformacionHomogeneaInversa([x(i+1),y(i+1),theta(i+1)],wp_vec(lane_id,:),wp_vec(lane_id+1,:));
    e_Lcurr(i)=P_localcurr(2);
    e_thetacurr(i)=P_localcurr(3);
    P1_localnext = transformacionHomogeneaInversa([x(i+1),y(i+1),theta(i+1)], wp_vec(wp_next_index, :), wp_vec(wp_next_next_index, :));
    e_Lnext(i)=P1_localnext(2);
    e_thetanext(i)=P1_localnext(3);
end
% Plot position evolution
hold on
plot(x,y,'LineWidth',1.2,'Color',color_selector(4),'DisplayName','Vehicle position')
hold on
plot(x(1),y(1),'-p', 'MarkerSize', 10, 'MarkerEdgeColor', 'yellow', 'MarkerFaceColor', 'yellow','DisplayName','Initial state')
hold on
plot(x(end),y(end),'-p', 'MarkerSize', 10, 'MarkerEdgeColor', 'green', 'MarkerFaceColor', 'green','DisplayName','Final state')
hold on
plot(wp_vec(:,1),wp_vec(:,2),'^', 'MarkerSize', 5, 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'black','DisplayName','WP_{i}')
hold off
axis([min(x)*1.1 max(x)*1.1 min(y)*1.1 max(y)*1.1])
legend;
xlabel('x (m)')
ylabel('y (m)')
% Plot safe indicators state space
figure
grass=[0.24705882352941178,0.40784313725490196,0.10980392156862745];
sky=[0.21568627450980393,0.3686274509803922,0.592156862745098];
shadow=[0.16470588235294117,0.19215686274509805,0.19607843137254902];
sunset=[0.984313725490196,0.396078431372549,0.25882352941176473];

% S i
load("eL0_bicycle_draw.mat");
load("theta0max_bicycle_draw.mat")
load("theta0min_bicycle_draw.mat")
plot(eL0_bicycle_draw,theta0min_bicycle_draw,'Color',grass,'LineWidth',1.2,'DisplayName','S')
hold on
plot(eL0_bicycle_draw,theta0max_bicycle_draw,'Color',grass,'LineWidth',1.2,'HandleVisibility','off')
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)], [theta0min_bicycle_draw(1) theta0max_bicycle_draw(1)],'Color',grass,'LineWidth',1.2,'HandleVisibility','off')
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)], [theta0min_bicycle_draw(end) theta0max_bicycle_draw(end)],'Color',grass,'LineWidth',1.2,'HandleVisibility','off')
hold on
% Sfw i
load("eL0_fwsat.mat")
load("thetamax_fwsat.mat")
load("thetamin_fwsat.mat")
plot(eL0_fwsat,thetamin_fwsat,'Color',sky,'LineWidth',1.2,'DisplayName','S_{fw}')
hold on
plot(eL0_fwsat,thetamax_fwsat,'Color',sky,'LineWidth',1.2,'HandleVisibility','off')
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)], [thetamin_fwsat(1) thetamax_fwsat(1)],'Color',sky,'LineWidth',1.2,'HandleVisibility','off')
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)], [thetamin_fwsat(end) thetamax_fwsat(end)],'Color',sky,'LineWidth',1.2,'HandleVisibility','off')
hold on
% C i
load("theta_fw_alpha_draw.mat")
plot(eL0_bicycle_draw,theta_fw_alpha(:,1),'Color',sunset,'LineWidth',1.2,'DisplayName','^{i} C_{i+1}')
hold on
plot(eL0_bicycle_draw,theta_fw_alpha(:,2),'Color',sunset,'LineWidth',1.2,'HandleVisibility','off')
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)], [theta_fw_alpha(1,1) theta_fw_alpha(1,2)],'Color',sunset,'LineWidth',1.2,'HandleVisibility','off')
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)], [theta_fw_alpha(end,1) theta_fw_alpha(end,2)],'Color',sunset,'LineWidth',1.2,'HandleVisibility','off')

% states i
e_Lcurr=max(-width/2, min(width/2, e_Lcurr));
plot(e_Lcurr(1:215),e_thetacurr(1:215),'-','Color',shadow,'LineWidth',1,'DisplayName','State e')
hold on
plot(e_Lcurr(1),e_thetacurr(1),'-p', 'MarkerSize', 10, 'MarkerEdgeColor', 'yellow', 'MarkerFaceColor', 'yellow','DisplayName','Initial state')
hold on
%plot(e_Lcurr(end),e_thetacurr(end),'-p', 'MarkerSize', 10, 'MarkerEdgeColor', 'green', 'MarkerFaceColor', 'green','DisplayName','Final state')
hold off
axis([min(eL0_bicycle_draw)*1.1 max(eL0_bicycle_draw)*1.1 min(theta0min_bicycle_draw)*1.1 max(theta0max_bicycle_draw)*1.1])
% Añadir leyenda
legend
xlabel('e_{L} (m)')
ylabel('e_{\theta} (rad)')

figure
plot(t, delta_drunk,'--', 'Color', color_selector(9),'LineWidth',1,'DisplayName','\delta_{swerve}')
hold on
plot(t, delta_grid_vec(:,1),'Color', color_selector(10),'LineWidth',1,'DisplayName','ASIF bounds')
hold on
plot(t, delta_grid_vec(:,2),'Color', color_selector(10),'LineWidth',1,'HandleVisibility','off')
hold on
plot(t, delta, 'Color', color_selector(8),'LineWidth',1,'DisplayName','ASIF \delta')

legend
xlabel('time (s)')
ylabel('\delta (rad)')
%%
figure
% S i
load("eL0_bicycle_draw.mat");
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
% Sfw i
load("eL0_fwsat.mat")
load("thetamax_fwsat.mat")
load("thetamin_fwsat.mat")
plot(eL0_fwsat,thetamin_fwsat,'b')
hold on
plot(eL0_fwsat,thetamax_fwsat,'b')
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'b')
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'b')
hold on
% C i
load("theta_fw_alpha_draw.mat")
plot(eL0_bicycle_draw,theta_fw_alpha(:,1),'r')
hold on
plot(eL0_bicycle_draw,theta_fw_alpha(:,2),'r')
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta_fw_alpha(1,1) theta_fw_alpha(1,2)],'r')
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta_fw_alpha(end,1) theta_fw_alpha(end,2)],'r')
hold on
%states i
eLp=zeros([1 20]);
ethetap=zeros([1 20]);
eLp(1)=-0.229129;
ethetap(1)=-theta_fwsat;
delta=delta_max;
for i=1:19
    eLp(i+1) = eLp(i) + (L*(cos(delta+ethetap(i))-cos(ethetap(i)+(V*delta*(Ts)/L)+delta))/sin(delta));
    ethetap(i+1) = V*delta*Ts/L + ethetap(i);
    plot(eLp(i),ethetap(i),'*');
    hold on
    if (-theta_fwsat+alpha)<=ethetap(i)
        sprintf('Iteration %d , the asif needs to force with a time interval of %f s', i, i*Ts)
        break
    end
end
%%
etheta_reach=zeros([1 20]);
etheta_reach(1)=-theta_fwsat+alpha;
for i=1:19
    etheta_reach(i+1)=etheta_reach(i)-(V*Ts*sin(delta_max))/L;
    el_izq=(-width/2)-((cos(etheta_reach(i+1)+delta_max)-1)*L/sin(delta_max));
    plot([el_izq width/2],[etheta_reach(i+1) etheta_reach(i+1)],'Color',color_selector(5))
    if etheta_reach(i+1)<=-theta_fwsat
        sprintf('Setp number %d , the asif needs to force with a time interval of %f s', i, i*Ts)
        break;
    end
end
hold off
%%
inc_theta=sin(pi/6)*V*Ts/L;
fwsath=pi/3;
alpha=pi/6;
int_v1=etheta_bounds_fw_alpha(0.3,fwsath,alpha,0.3);
(-fwsath-int_v1(1))/inc_theta;
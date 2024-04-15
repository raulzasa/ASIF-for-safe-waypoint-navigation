%% TUTORIAL INTLAB

A = intval([0.1,2;3,4]) % Allows floating point interval matrix

A = intval('0.1 2 3 4') %using string

rad(A(1,1)) % nonzero radius

A = reshape(A,2,2) % change column vector to matrix

B = midrad([1,0;0.5,3],1e-4) % give interval by its midpoint and radius

C = infsup([-1,0;2,4],[-0.9,0;2.4,4.01]) % an interval may be input by its infimum and supremum


% Crear intervalos
a = infsup(1, 2) % Crea un intervalo desde 1 a 2
b = infsup(3, 4) % Crea otro intervalo desde 3 a 4

% Operaciones básicas con intervalos
c = a + b % Suma de intervalos
d = a * b % Producto de intervalos

sin(a)
%%

%[x]=x([t],x0,[u])
% x0 --> initial state x(t=0)=x0
% [t] --> time interval [0,Ts]
% [u] --> control action interval WHAT IS DESIRED TO OBTAIN

% data
V=1;
L=0.264;
lane_width=0.6;
delta_max=pi/6;
delta_min=-delta_max;
theta_fwsat=pi/3;
Ts=0.1;
% intervals
delta_int=infsup(delta_min,delta_max);
t_int=infsup(0,Ts);
eL_int=infsup(-lane_width/2,lane_width/2);
% intial states
e_theta=pi/3;
e_L=0.15;
% interval projection
e_theta_next = V*sin(delta_int)*t_int/L + e_theta
e_L_next = e_L + (L*(cos(delta_int+e_theta)-cos(e_theta+(V*sin(delta_int)*t_int/L)+delta_int))/sin(delta_int));
e_L_next1 = (eL_int-e_L)*sin(delta_int)
e_L_next2 = L*(cos(delta_int+e_theta)-cos(e_theta+(V*sin(delta_int)*t_int/L)+delta_int))
intersect(e_L_next1,e_L_next2)
%%
% Definición de los intervalos excluyendo el 0
delta_int_neg = infsup(delta_min, -eps); % Un poco antes de 0 para excluirlo
delta_int_pos = infsup(eps, delta_max);  % Un poco después de 0 para excluirlo

% Cálculos para cada intervalo
% Para delta_int_neg
e_theta_next_neg = e_theta + V*sin(delta_int_neg)*t_int/L
e_L_next_neg = e_L + (L*(cos(delta_int_neg + e_theta) - cos(e_theta + (V*sin(delta_int_neg)*t_int)/L + delta_int_neg)))/sin(delta_int_neg)

% Para delta_int_pos
e_theta_next_pos = e_theta + V*sin(delta_int_pos)*t_int/L
e_L_next_pos = e_L + (L*(cos(delta_int_pos + e_theta) - cos(e_theta + (V*sin(delta_int_pos)*t_int)/L + delta_int_pos)))/sin(delta_int_pos)


%%
% data
V=1;
L=0.264;
lane_width=0.6;
delta_max=pi/6;
delta_min=-delta_max;
alpha=pi/6;
theta_fwsat=pi/3;
Ts=0.1;
% intervals
delta_int=infsup(delta_min,delta_max);
t_int=infsup(eps,Ts);
eL_int=infsup(-lane_width/2,lane_width/2);
% intial states
e_theta=pi/3;
e_L=0.15;
% interval projection
e_theta_next = V*sin(delta_min)*t_int/L + e_theta
% e_L_next = e_L + (L*(cos(delta_int+e_theta)-cos(e_theta+(V*sin(delta_int)*t_int/L)+delta_int))/sin(delta_int))
e_L_next1 = e_L + (L*(cos(delta_max+e_theta)-cos(e_theta+(V*sin(delta_max)*t_int/L)+delta_max))/sin(delta_max))
e_L_next3 = e_L + V*sin(e_theta)*t_int
e_L_next2 = e_L + (L*(cos(delta_min+e_theta)-cos(e_theta+(V*sin(delta_min)*t_int/L)+delta_min))/sin(delta_min))
e_L_next_sup = max([sup(e_L_next1),sup(e_L_next2),sup(e_L_next3)])
e_L_next_inf = min([sup(e_L_next1),sup(e_L_next2),sup(e_L_next3)])
e_theta_next_sup = etheta_bounds_fw_alpha(e_L_next_sup,theta_fwsat,alpha,lane_width/2)
e_theta_next_inf = etheta_bounds_fw_alpha(e_L_next_inf,theta_fwsat,alpha,lane_width/2)
e_theta_next_intersect = intersect(infsup(e_theta_next_sup(1),e_theta_next_sup(2)),infsup(e_theta_next_inf(1),e_theta_next_inf(2)))
a=asin(sup((e_theta_next_intersect - e_theta)*(L/V)/t_int))
delta_intmod=infsup(delta_min,a)
e_theta_next = V*sin(delta_intmod)*t_int/L + e_theta


%%
% data
V=1;
L=0.264;
lane_width=0.6;
delta_max=pi/6;
delta_min=-delta_max;
alpha=pi/6;
theta_fwsat=pi/3;
Ts=0.1;
incdelt=(delta_max-delta_min)/41;
% intervals
delta_int=infsup(delta_min,delta_max);
t_int=infsup(eps,Ts);
eL_int=infsup(-lane_width/2,lane_width/2);
delta_intmax=infsup(delta_max,delta_max)
delta_intmin=infsup(delta_min,delta_min)
% intial states
e_theta=pi/3;
e_L=0.15;
% interval projection
% Starting on deltamax
e_theta_nextmax = V*sin(delta_intmax)*t_int/L + e_theta
e_L_nextmax = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*t_int/L)+delta_intmax))/sin(delta_intmax))
% Starting on deltamin
e_theta_nextmin = V*sin(delta_intmin)*t_int/L + e_theta
e_L_nextmin = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*t_int/L)+delta_intmin))/sin(delta_intmin))

% Plot safe indicators state space
figure
% S i
% load("eL0_bicycle_draw.mat");
% load("theta0max_bicycle_draw.mat")
% load("theta0min_bicycle_draw.mat")
% plot(eL0_bicycle_draw,theta0min_bicycle_draw,'g')
% hold on
% plot(eL0_bicycle_draw,theta0max_bicycle_draw,'g')
% hold on
% plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta0min_bicycle_draw(1) theta0max_bicycle_draw(1)],'g')
% hold on
% plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta0min_bicycle_draw(end) theta0max_bicycle_draw(end)],'g')
% hold on
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
% load("theta_fw_alpha_draw.mat")
% plot(eL0_bicycle_draw,theta_fw_alpha(:,1),'r')
% hold on
% plot(eL0_bicycle_draw,theta_fw_alpha(:,2),'r')
% hold on
% plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)],[theta_fw_alpha(1,1) theta_fw_alpha(1,2)],'r')
% hold on
% plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)],[theta_fw_alpha(end,1) theta_fw_alpha(end,2)],'r')
% hold on
% states i
if isInvariantSet_fwsat(sup(e_L_nextmax),sup(e_theta_nextmax),theta_fwsat)
    rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), sup(e_L_nextmax)-inf(e_L_nextmax), sup(e_theta_nextmax)-inf(e_theta_nextmax)], 'EdgeColor', color_selector(1), 'LineWidth', 1)
    hold on
    for i = 1:10
        delta_intmax = infsup(delta_max-incdelt*i,delta_max);
        e_theta_nextmax = V*sin(delta_intmax)*t_int/L + e_theta
        e_L_nextmax = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*t_int/L)+delta_intmax))/sin(delta_intmax))
        rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), sup(e_L_nextmax)-inf(e_L_nextmax), sup(e_theta_nextmax)-inf(e_theta_nextmax)], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
        hold on
    end
elseif isInvariantSet_fwsat(sup(e_L_nextmin),sup(e_theta_nextmin),theta_fwsat)
    rectangle('Position', [inf(e_L_nextmin), inf(e_theta_nextmin), sup(e_L_nextmin)-inf(e_L_nextmin), sup(e_theta_nextmin)-inf(e_theta_nextmin)], 'EdgeColor', color_selector(1), 'LineWidth', 1)
    hold on
    for i = 1:10
        delta_intmin = infsup(delta_min,delta_min+incdelt*i)
        e_theta_nextmin = V*sin(delta_intmin)*t_int/L + e_theta
        e_L_nextmin = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*t_int/L)+delta_intmin))/sin(delta_intmin))
        rectangle('Position', [inf(e_L_nextmin), inf(e_theta_nextmin), sup(e_L_nextmin)-inf(e_L_nextmin), sup(e_theta_nextmin)-inf(e_theta_nextmin)], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)        
        hold on
    end
else
    rectangle('Position', [inf(e_L_nextmin), inf(e_theta_nextmin), sup(e_L_nextmin)-inf(e_L_nextmin), sup(e_theta_nextmin)-inf(e_theta_nextmin)], 'EdgeColor', color_selector(1), 'LineWidth', 1)
    hold on
    for i = 1:10
        delta_intmin = infsup(delta_min,delta_min+incdelt*i);
        e_theta_nextmax = V*sin(delta_intmin)*t_int/L + e_theta
        e_L_nextmax = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*t_int/L)+delta_intmin))/sin(delta_intmin))
        rectangle('Position', [inf(e_L_nextmin), inf(e_theta_nextmin), sup(e_L_nextmin)-inf(e_L_nextmin), sup(e_theta_nextmin)-inf(e_theta_nextmin)], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)        
        hold on
    end
    rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), sup(e_L_nextmax)-inf(e_L_nextmax), sup(e_theta_nextmax)-inf(e_theta_nextmax)], 'EdgeColor', color_selector(1), 'LineWidth', 1)
    hold on
    for i = 1:10
        delta_intmax = infsup(delta_max-incdelt*i,delta_max);
        e_theta_nextmax = V*sin(delta_intmax)*t_int/L + e_theta
        e_L_nextmax = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*t_int/L)+delta_intmax))/sin(delta_intmax))
        rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), sup(e_L_nextmax)-inf(e_L_nextmax), sup(e_theta_nextmax)-inf(e_theta_nextmax)], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
        hold on
    end
end
hold off

%%
% data
V=1;
L=0.264;
lane_width=0.6;
delta_max=pi/6;
delta_min=-delta_max;
alpha=pi/6;
theta_fwsat=pi/3;
Ts=0.1;
incdelt=(delta_max-delta_min)/21;
% intervals
delta_int=infsup(delta_min,delta_max);
%t_int=infsup(eps,Ts);
 t_int=Ts;
eL_int=infsup(-lane_width/2,lane_width/2);
delta_intmax=infsup(delta_max,delta_max)
delta_intmin=infsup(delta_min,delta_min)
% intial states
e_theta=0;
e_L=0;
% interval projection
% Starting on deltamin
e_theta_nextmin = V*sin(delta_intmin)*t_int/L + e_theta
e_L_nextmin = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*t_int/L)+delta_intmin))/sin(delta_intmin))

% Plot safe indicators state space
figure

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

plot(e_L,e_theta,'ok')
rectangle('Position', [inf(e_L_nextmin), inf(e_theta_nextmin), (sup(e_L_nextmin)-inf(e_L_nextmin)), (sup(e_theta_nextmin)-inf(e_theta_nextmin))], 'EdgeColor', color_selector(1), 'LineWidth', 1)
hold on
e_theta_nextval = V*sin(delta_min)*Ts/L + e_theta
e_L_nextval = e_L + (L*(cos(delta_min+e_theta)-cos(e_theta+(V*sin(delta_min)*Ts/L)+delta_min))/sin(delta_min))
plot(e_L_nextval,e_theta_nextval,'*k')
hold on
for i = 1:10
    delta_intmin = infsup(delta_min,delta_min+incdelt*i)
    e_theta_nextmin = V*sin(delta_intmin)*t_int/L + e_theta
    e_L_nextmin = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*t_int/L)+delta_intmin))/sin(delta_intmin))
    % rectangle('Position', [e_L, e_theta-(sup(e_theta_nextmin)-inf(e_theta_nextmin)), sup(e_L_nextmin)-e_L, (sup(e_theta_nextmin)-inf(e_theta_nextmin))], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
    rectangle('Position', [inf(e_L_nextmin), inf(e_theta_nextmin), (sup(e_L_nextmin)-inf(e_L_nextmin)), (sup(e_theta_nextmin)-inf(e_theta_nextmin))], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
    hold on
    delta_valmin=delta_min+incdelt*i;
    e_theta_nextval = V*sin(delta_valmin)*Ts/L + e_theta
    e_L_nextval = e_L + (L*(cos(delta_valmin+e_theta)-cos(e_theta+(V*sin(delta_valmin)*Ts/L)+delta_valmin))/sin(delta_valmin))
    plot(e_L_nextval,e_theta_nextval,'*k')
    hold on
end

hold on

% interval projection
% Starting on deltamin
e_theta_nextmax = V*sin(delta_intmax)*t_int/L + e_theta
e_L_nextmax = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*t_int/L)+delta_intmax))/sin(delta_intmax))

% Plot safe indicators state space


plot(e_L,e_theta,'ok')
rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), (sup(e_L_nextmax)-inf(e_L_nextmax)), (sup(e_theta_nextmax)-inf(e_theta_nextmax))], 'EdgeColor', color_selector(1), 'LineWidth', 1)
hold on
e_theta_nextval = V*sin(delta_max)*Ts/L + e_theta
e_L_nextval = e_L + (L*(cos(delta_max+e_theta)-cos(e_theta+(V*sin(delta_max)*Ts/L)+delta_max))/sin(delta_max))
plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(1))
hold on
for i = 1:10
    delta_intmax = infsup(delta_max-incdelt*i,delta_max)
    e_theta_nextmax = V*sin(delta_intmax)*t_int/L + e_theta
    e_L_nextmax = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*t_int/L)+delta_intmax))/sin(delta_intmax))
    % rectangle('Position', [e_L, e_theta-(sup(e_theta_nextmin)-inf(e_theta_nextmin)), sup(e_L_nextmin)-e_L, (sup(e_theta_nextmin)-inf(e_theta_nextmin))], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
    rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), (sup(e_L_nextmax)-inf(e_L_nextmax)), (sup(e_theta_nextmax)-inf(e_theta_nextmax))], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
    hold on
    delta_valmax=delta_max-incdelt*i;
    e_theta_nextval = V*sin(delta_valmax)*Ts/L + e_theta
    e_L_nextval = e_L + (L*(cos(delta_valmax+e_theta)-cos(e_theta+(V*sin(delta_valmax)*Ts/L)+delta_valmax))/sin(delta_valmax))
    plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(i+1))
    hold on
end

hold off
%%
% data
V=1;
L=0.264;
lane_width=0.6;
delta_max=pi/6;
delta_min=-delta_max;
alpha=pi/6;
theta_fwsat=pi/3;
Ts=0.1;
incdelt=(delta_max-delta_min)/21;
% intervals
%delta_int=infsup(delta_min,delta_max);
%t_int=infsup(eps,Ts);
% t_int=Ts;
% eL_int=infsup(-lane_width/2,lane_width/2);
% delta_intmax=infsup(delta_max,delta_max)
% delta_intmin=infsup(delta_min,delta_min)
% intial states
% e_theta=0;  m, e_theta(0) = 
% e_theta=-1.0472; % thetafw
e_theta=0.5 ;
e_L=-0.3;
% interval projection
% Starting on deltamin
% e_theta_nextmin = V*sin(delta_intmin)*t_int/L + e_theta
% e_L_nextmin = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*t_int/L)+delta_intmin))/sin(delta_intmin))

tspan = [0, Ts];  % Intervalo de tiempo
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
% Plot safe indicators state space
figure

load("eL0_fwsat.mat")
load("thetamax_fwsat.mat")
load("thetamin_fwsat.mat")
plot(eL0_fwsat,thetamin_fwsat,'k','LineWidth',1)
hold on
plot(eL0_fwsat,thetamax_fwsat,'k','LineWidth',1)
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'k','LineWidth',1)
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'k','LineWidth',1)
hold on

delta_min=-e_theta
% plot(e_L,e_theta,'ok')
% rectangle('Position', [inf(e_L_nextmin), inf(e_theta_nextmin), (sup(e_L_nextmin)-inf(e_L_nextmin)), (sup(e_theta_nextmin)-inf(e_theta_nextmin))], 'EdgeColor', color_selector(1), 'LineWidth', 1)
% hold on
e_theta_nextval = V*sin(delta_min)*Ts/L + e_theta
e_L_nextval = e_L + (L*(cos(delta_min+e_theta)-cos(e_theta+(V*sin(delta_min)*Ts/L)+delta_min))/sin(delta_min))
plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(1))
hold on
[t, x] = ode45(@sistema_dinamico, tspan, [e_L; e_theta], options, V, L, delta_min);
plot(x(:,1),x(:,2),'Color',color_selector(1))
hold on
for i = 1:10
    delta_intmin = infsup(delta_min+incdelt*(i-1),delta_min+incdelt*i)
    e_theta_nextmin = V*sin(delta_intmin)*t_int/L + e_theta
    e_L_nextmin = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*t_int/L)+delta_intmin))/sin(delta_intmin))
    % rectangle('Position', [e_L, e_theta-(sup(e_theta_nextmin)-inf(e_theta_nextmin)), sup(e_L_nextmin)-e_L, (sup(e_theta_nextmin)-inf(e_theta_nextmin))], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
    rectangle('Position', [inf(e_L_nextmin), inf(e_theta_nextmin), (sup(e_L_nextmin)-inf(e_L_nextmin)), (sup(e_theta_nextmin)-inf(e_theta_nextmin))], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
    hold on
    delta_valmin=delta_min+incdelt*i;
    e_theta_nextval = V*sin(delta_valmin)*Ts/L + e_theta
    e_L_nextval = e_L + (L*(cos(delta_valmin+e_theta)-cos(e_theta+(V*sin(delta_valmin)*Ts/L)+delta_valmin))/sin(delta_valmin))
    plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(i+1))
    hold on
    [t, x] = ode45(@sistema_dinamico, tspan, [e_L; e_theta], options, V, L, delta_valmin);
    plot(x(:,1),x(:,2),'Color',color_selector(i+1))
    hold on
end

hold on

% interval projection
% Starting on deltamin
e_theta_nextmax = V*sin(delta_intmax)*t_int/L + e_theta
e_L_nextmax = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*t_int/L)+delta_intmax))/sin(delta_intmax))

% Plot safe indicators state space


plot(e_L,e_theta,'ok')
rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), (sup(e_L_nextmax)-inf(e_L_nextmax)), (sup(e_theta_nextmax)-inf(e_theta_nextmax))], 'EdgeColor', color_selector(1), 'LineWidth', 1)
hold on
e_theta_nextval = V*sin(delta_max)*Ts/L + e_theta
e_L_nextval = e_L + (L*(cos(delta_max+e_theta)-cos(e_theta+(V*sin(delta_max)*Ts/L)+delta_max))/sin(delta_max))
plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(1))
hold on
[t, x] = ode45(@sistema_dinamico, tspan, [e_L; e_theta], options, V, L, delta_max);
plot(x(:,1),x(:,2),'Color',color_selector(1))
hold on
for i = 1:10
    delta_intmax = infsup(delta_max-incdelt*i,delta_max-incdelt*(i-1))
    e_theta_nextmax = V*sin(delta_intmax)*t_int/L + e_theta
    e_L_nextmax = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*t_int/L)+delta_intmax))/sin(delta_intmax))
    % rectangle('Position', [e_L, e_theta-(sup(e_theta_nextmin)-inf(e_theta_nextmin)), sup(e_L_nextmin)-e_L, (sup(e_theta_nextmin)-inf(e_theta_nextmin))], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
    rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), (sup(e_L_nextmax)-inf(e_L_nextmax)), (sup(e_theta_nextmax)-inf(e_theta_nextmax))], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
    hold on
    delta_valmax=delta_max-incdelt*i;
    e_theta_nextval = V*sin(delta_valmax)*Ts/L + e_theta
    e_L_nextval = e_L + (L*(cos(delta_valmax+e_theta)-cos(e_theta+(V*sin(delta_valmax)*Ts/L)+delta_valmax))/sin(delta_valmax))
    plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(i+1))
    hold on
    [t, x] = ode45(@sistema_dinamico, tspan, [e_L; e_theta], options, V, L, delta_valmax);
    plot(x(:,1),x(:,2),'Color',color_selector(i+1))
    hold on
end
xlabel('Lateral error (e_{L})')
ylabel('Yaw error (e_{\theta})')
hold off

%%
% data
V=1;
L=0.264;
lane_width=0.6;
delta_max=pi/6;
delta_min=-delta_max;
alpha=pi/6;
theta_fwsat=pi/3;
Ts=0.1;
incdelt=(delta_max-delta_min)/21;
% intervals
delta_int=infsup(delta_min,delta_max);
%t_int=infsup(eps,Ts);
 t_int=Ts;
eL_int=infsup(-lane_width/2,lane_width/2);
delta_intmax=infsup(delta_max,delta_max)
delta_intmin=infsup(delta_min,delta_min)
% intial states
e_theta=0;
e_L=0;
% interval projection
% Starting on deltamin
e_theta_nextmin = V*sin(delta_intmin)*t_int/L + e_theta
e_L_nextmin = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*t_int/L)+delta_intmin))/sin(delta_intmin))

% Plot safe indicators state space
figure

load("eL0_fwsat.mat")
load("thetamax_fwsat.mat")
load("thetamin_fwsat.mat")
plot(eL0_fwsat,thetamin_fwsat,'k','LineWidth',1)
hold on
plot(eL0_fwsat,thetamax_fwsat,'k','LineWidth',1)
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'k','LineWidth',1)
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'k','LineWidth',1)
hold on

plot(e_L,e_theta,'ok')
rectangle('Position', [inf(e_L_nextmin), inf(e_theta_nextmin), (sup(e_L_nextmin)-inf(e_L_nextmin)), (sup(e_theta_nextmin)-inf(e_theta_nextmin))], 'EdgeColor', color_selector(1), 'LineWidth', 1)
hold on
e_theta_nextval = V*sin(delta_min)*Ts/L + e_theta
e_L_nextval = e_L + (L*(cos(delta_min+e_theta)-cos(e_theta+(V*sin(delta_min)*Ts/L)+delta_min))/sin(delta_min))
plot(e_L_nextval,e_theta_nextval,'*k')
hold on
for i = 1:10
    delta_intmin = infsup(delta_min,delta_min+incdelt*i)
    e_theta_nextmin = V*sin(delta_intmin)*t_int/L + e_theta
    e_L_nextmin = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*t_int/L)+delta_intmin))/sin(delta_intmin))
    % rectangle('Position', [e_L, e_theta-(sup(e_theta_nextmin)-inf(e_theta_nextmin)), sup(e_L_nextmin)-e_L, (sup(e_theta_nextmin)-inf(e_theta_nextmin))], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
    rectangle('Position', [inf(e_L_nextmin), inf(e_theta_nextmin), (sup(e_L_nextmin)-inf(e_L_nextmin)), (sup(e_theta_nextmin)-inf(e_theta_nextmin))], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
    hold on
    delta_valmin=delta_min+incdelt*i;
    e_theta_nextval = V*sin(delta_valmin)*Ts/L + e_theta
    e_L_nextval = e_L + (L*(cos(delta_valmin+e_theta)-cos(e_theta+(V*sin(delta_valmin)*Ts/L)+delta_valmin))/sin(delta_valmin))
    plot(e_L_nextval,e_theta_nextval,'*k')
    hold on
end

hold on

% interval projection
% Starting on deltamin
e_theta_nextmax = V*sin(delta_intmax)*t_int/L + e_theta
e_L_nextmax = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*t_int/L)+delta_intmax))/sin(delta_intmax))

% Plot safe indicators state space


plot(e_L,e_theta,'ok')
rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), (sup(e_L_nextmax)-inf(e_L_nextmax)), (sup(e_theta_nextmax)-inf(e_theta_nextmax))], 'EdgeColor', color_selector(1), 'LineWidth', 1)
hold on
e_theta_nextval = V*sin(delta_max)*Ts/L + e_theta
e_L_nextval = e_L + (L*(cos(delta_max+e_theta)-cos(e_theta+(V*sin(delta_max)*Ts/L)+delta_max))/sin(delta_max))
plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(1))
hold on
for i = 1:10
    delta_intmax = infsup(delta_max-incdelt*i,delta_max)
    e_theta_nextmax = V*sin(delta_intmax)*t_int/L + e_theta
    e_L_nextmax = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*t_int/L)+delta_intmax))/sin(delta_intmax))
    % rectangle('Position', [e_L, e_theta-(sup(e_theta_nextmin)-inf(e_theta_nextmin)), sup(e_L_nextmin)-e_L, (sup(e_theta_nextmin)-inf(e_theta_nextmin))], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
    rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), (sup(e_L_nextmax)-inf(e_L_nextmax)), (sup(e_theta_nextmax)-inf(e_theta_nextmax))], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
    hold on
    delta_valmax=delta_max-incdelt*i;
    e_theta_nextval = V*sin(delta_valmax)*Ts/L + e_theta
    e_L_nextval = e_L + (L*(cos(delta_valmax+e_theta)-cos(e_theta+(V*sin(delta_valmax)*Ts/L)+delta_valmax))/sin(delta_valmax))
    plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(i+1))
    hold on
end
xlabel('Lateral error (e_{L})')
ylabel('Yaw error (e_{\theta})')
hold off

%%
% data
V=1;
L=0.264;
lane_width=0.6;
delta_max=pi/6;
delta_min=-delta_max;
alpha=pi/6;
theta_fwsat=pi/3;
Ts=0.1;
incdelt=(delta_max-delta_min)/21;
% intervals
delta_int=infsup(delta_min,delta_max);
t_int=infsup(eps,Ts);
 % t_int=Ts;
eL_int=infsup(-lane_width/2,lane_width/2);
delta_intmax=delta_max
delta_intmin=delta_min
% intial states
% e_theta=1.0472; % thetafw
% e_theta=pi/6;
e_theta=0.6;
% e_L=0.229129; %vertice thetafw con upper ultimate safe trajectory
e_L=-0.3; %thetafw
% e_L=0;
% e_L = 0.262763;
% e_theta = 0.901408;
% interval projection
% Starting on deltamin
e_theta_nextmin = V*sin(delta_intmin)*t_int/L + e_theta
e_L_nextmin = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*t_int/L)+delta_intmin))/sin(delta_intmin))

tspan = [0, Ts];  % Intervalo de tiempo
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

% Plot safe indicators state space
figure

load("eL0_fwsat.mat")
load("thetamax_fwsat.mat")
load("thetamin_fwsat.mat")
plot(eL0_fwsat,thetamin_fwsat,'k','LineWidth',1)
hold on
plot(eL0_fwsat,thetamax_fwsat,'k','LineWidth',1)
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'k','LineWidth',1)
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'k','LineWidth',1)
hold on

plot(e_L,e_theta,'ok')
rectangle('Position', [inf(e_L_nextmin), inf(e_theta_nextmin), (sup(e_L_nextmin)-inf(e_L_nextmin)), (sup(e_theta_nextmin)-inf(e_theta_nextmin))], 'EdgeColor', color_selector(1), 'LineWidth', 1)
hold on
e_theta_nextval = V*sin(delta_min)*Ts/L + e_theta
e_L_nextval = e_L + (L*(cos(delta_min+e_theta)-cos(e_theta+(V*sin(delta_min)*Ts/L)+delta_min))/sin(delta_min))
plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(1))
hold on
[t, x] = ode45(@sistema_dinamico, tspan, [e_L; e_theta], options, V, L, delta_min);
plot(x(:,1),x(:,2),'Color',color_selector(1))
hold on
for i = 1:10
    delta_intmin = delta_min+incdelt*i
    e_theta_nextmin = V*sin(delta_intmin)*t_int/L + e_theta
    e_L_nextmin = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*t_int/L)+delta_intmin))/sin(delta_intmin))
    % rectangle('Position', [e_L, e_theta-(sup(e_theta_nextmin)-inf(e_theta_nextmin)), sup(e_L_nextmin)-e_L, (sup(e_theta_nextmin)-inf(e_theta_nextmin))], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
    rectangle('Position', [inf(e_L_nextmin), inf(e_theta_nextmin), (sup(e_L_nextmin)-inf(e_L_nextmin)), (sup(e_theta_nextmin)-inf(e_theta_nextmin))], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
    hold on
    delta_valmin=delta_min+incdelt*i;
    e_theta_nextval = V*sin(delta_valmin)*Ts/L + e_theta
    e_L_nextval = e_L + (L*(cos(delta_valmin+e_theta)-cos(e_theta+(V*sin(delta_valmin)*Ts/L)+delta_valmin))/sin(delta_valmin))
    plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(i+1))
    hold on
    

    % Resolver el sistema dinámico con ode45
    [t, x] = ode45(@sistema_dinamico, tspan, [e_L; e_theta], options, V, L, delta_intmin);
    plot(x(:,1),x(:,2),'Color',color_selector(i+1))
end

hold on

% interval projection
% Starting on deltamin
e_theta_nextmax = V*sin(delta_intmax)*t_int/L + e_theta
e_L_nextmax = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*t_int/L)+delta_intmax))/sin(delta_intmax))

% Plot safe indicators state space


plot(e_L,e_theta,'ok')
rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), (sup(e_L_nextmax)-inf(e_L_nextmax)), (sup(e_theta_nextmax)-inf(e_theta_nextmax))], 'EdgeColor', color_selector(1), 'LineWidth', 1)
hold on
e_theta_nextval = V*sin(delta_max)*Ts/L + e_theta
e_L_nextval = e_L + (L*(cos(delta_max+e_theta)-cos(e_theta+(V*sin(delta_max)*Ts/L)+delta_max))/sin(delta_max))
plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(1))
hold on
[t, x] = ode45(@sistema_dinamico, tspan, [e_L; e_theta], options, V, L, delta_max);
plot(x(:,1),x(:,2),'Color',color_selector(1))
hold on
for i = 1:10
    delta_intmax = delta_max-incdelt*i
    e_theta_nextmax = V*sin(delta_intmax)*t_int/L + e_theta
    e_L_nextmax = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*t_int/L)+delta_intmax))/sin(delta_intmax))
    % rectangle('Position', [e_L, e_theta-(sup(e_theta_nextmin)-inf(e_theta_nextmin)), sup(e_L_nextmin)-e_L, (sup(e_theta_nextmin)-inf(e_theta_nextmin))], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
    rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), (sup(e_L_nextmax)-inf(e_L_nextmax)), (sup(e_theta_nextmax)-inf(e_theta_nextmax))], 'EdgeColor', color_selector(i+1), 'LineWidth', 1)
    hold on
    delta_valmax=delta_max-incdelt*i;
    e_theta_nextval = V*sin(delta_valmax)*Ts/L + e_theta
    e_L_nextval = e_L + (L*(cos(delta_valmax+e_theta)-cos(e_theta+(V*sin(delta_valmax)*Ts/L)+delta_valmax))/sin(delta_valmax))
    plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(i+1))
    hold on
    % Resolver el sistema dinámico con ode45
    [t, x] = ode45(@sistema_dinamico, tspan, [e_L; e_theta], options, V, L, delta_intmax);
    plot(x(:,1),x(:,2),'Color',color_selector(i+1))
end
xlabel('Lateral error (e_{L})')
ylabel('Yaw error (e_{\theta})')
hold off

%%
% data
V=1;
L=0.264;
lane_width=0.6;
delta_max=pi/6;
delta_min=-delta_max;
alpha=pi/6;
theta_fwsat=pi/3;
Ts=0.1;
incdelt=(delta_max-delta_min)/21;
% intervals
delta_int=infsup(delta_min,delta_max);
t_int=infsup(eps,Ts);
 % t_int=Ts;
eL_int=infsup(-lane_width/2,lane_width/2);

% intial states
% e_theta_negs = [-pi/6, -0.5, -0.4, -0.3, -0.2, -0.1];
e_theta_negs = [pi/6, 0.5, 0.4, 0.3, 0.2, 0.1];
% e_theta_negs = [theta_fwsat, 1, 0.7, 0.6];

for j=1:length(e_theta_negs)
% e_theta=1.0472; % thetafw
e_theta=e_theta_negs(j);
% e_theta=0;
% e_L=0.229129; %vertice thetafw con upper ultimate safe trajectory
e_L=-0.3+0.001; %thetafw
% e_L=0;
% e_L = 0.262763;
% e_theta = 0.901408;
% interval projection
% Starting on deltamin
% Plot safe indicators state space
figure

load("eL0_fwsat.mat")
load("thetamax_fwsat.mat")
load("thetamin_fwsat.mat")
plot(eL0_fwsat,thetamin_fwsat,'k','LineWidth',1)
hold on
plot(eL0_fwsat,thetamax_fwsat,'k','LineWidth',1)
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'k','LineWidth',1)
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'k','LineWidth',1)
hold on

tspan = [0, Ts];  % Intervalo de tiempo
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

delta_intmax=delta_max
% interval projection
% Starting on deltamin
e_theta_nextmax = V*sin(delta_intmax)*t_int/L + e_theta
e_L_nextmax = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*t_int/L)+delta_intmax))/sin(delta_intmax))
% Plot safe indicators state space
plot(e_L,e_theta,'ok')
rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), (sup(e_L_nextmax)-inf(e_L_nextmax)), (sup(e_theta_nextmax)-inf(e_theta_nextmax))], 'EdgeColor', color_selector(1), 'LineWidth', 1)
hold on
e_theta_nextval = V*sin(delta_intmax)*Ts/L + e_theta
e_L_nextval = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*Ts/L)+delta_intmax))/sin(delta_intmax))
plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(1))
hold on
[t, x] = ode45(@sistema_dinamico, tspan, [e_L; e_theta], options, V, L, delta_intmax);
plot(x(:,1),x(:,2),'Color',color_selector(1))
hold on

A = 2*L/(V*Ts); % Sustituye con el valor deseado
B = e_theta; % Sustituye con el valor deseado

% Definir la función que queremos resolver
func = @(x) sin(x) + A * (B + x);

% Estimación inicial para x
x0 = 0;

% Usar fzero para encontrar una raíz de la función 
% delta_intmax=-e_theta;
delta_intmax= fzero(func, x0);
e_theta_nextmax = V*sin(delta_intmax)*t_int/L + e_theta
e_L_nextmax = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*t_int/L)+delta_intmax))/sin(delta_intmax))
% Plot safe indicators state space
plot(e_L,e_theta,'ok')
rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), (sup(e_L_nextmax)-inf(e_L_nextmax)), (sup(e_theta_nextmax)-inf(e_theta_nextmax))], 'EdgeColor', color_selector(2), 'LineWidth', 1)
hold on
e_theta_nextval = V*sin(delta_intmax)*Ts/L + e_theta
e_L_nextval = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*Ts/L)+delta_intmax))/sin(delta_intmax))
plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(2))
hold on
[t, x] = ode45(@sistema_dinamico, tspan, [e_L; e_theta], options, V, L, delta_intmax);
plot(x(:,1),x(:,2),'Color',color_selector(2))
hold on
xlabel('Lateral error (e_{L})')
ylabel('Yaw error (e_{\theta})')
hold off
end

%%
% data
V=1;
L=0.264;
lane_width=0.6;
delta_max=pi/6;
delta_min=-delta_max;
alpha=pi/6;
theta_fwsat=pi/3;
Ts=0.1;
incdelt=(delta_max-delta_min)/21;
% intervals
delta_int=infsup(delta_min,delta_max);
t_int=infsup(eps,Ts);
 % t_int=Ts;
eL_int=infsup(-lane_width/2,lane_width/2);

% intial states
e_theta_negs = [pi/6, 0.5, 0.4, 0.3, 0.2, 0.1];
for j=1:length(e_theta_negs)
% e_theta=1.0472; % thetafw
e_theta=e_theta_negs(j);
% e_theta=0;
% e_L=0.229129; %vertice thetafw con upper ultimate safe trajectory
e_L=0.3; %thetafw
% e_L=0;
% e_L = 0.262763;
% e_theta = 0.901408;
% interval projection
% Starting on deltamin
% Plot safe indicators state space
figure

load("eL0_fwsat.mat")
load("thetamax_fwsat.mat")
load("thetamin_fwsat.mat")
plot(eL0_fwsat,thetamin_fwsat,'k','LineWidth',1)
hold on
plot(eL0_fwsat,thetamax_fwsat,'k','LineWidth',1)
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'k','LineWidth',1)
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'k','LineWidth',1)
hold on

tspan = [0, Ts];  % Intervalo de tiempo
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

delta_intmin=delta_min
% interval projection
% Starting on deltamin
e_theta_nextmax = V*sin(delta_intmin)*t_int/L + e_theta
e_L_nextmax = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*t_int/L)+delta_intmin))/sin(delta_intmin))
% Plot safe indicators state space
plot(e_L,e_theta,'ok')
rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), (sup(e_L_nextmax)-inf(e_L_nextmax)), (sup(e_theta_nextmax)-inf(e_theta_nextmax))], 'EdgeColor', color_selector(1), 'LineWidth', 1)
hold on
e_theta_nextval = V*sin(delta_intmin)*Ts/L + e_theta
e_L_nextval = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*Ts/L)+delta_intmin))/sin(delta_intmin))
plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(1))
hold on
[t, x] = ode45(@sistema_dinamico, tspan, [e_L; e_theta], options, V, L, delta_intmin);
plot(x(:,1),x(:,2),'Color',color_selector(1))
hold on

delta_intmin=-e_theta;
e_theta_nextmax = V*sin(delta_intmin)*t_int/L + e_theta
e_L_nextmax = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*t_int/L)+delta_intmin))/sin(delta_intmin))
% Plot safe indicators state space
plot(e_L,e_theta,'ok')
rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), (sup(e_L_nextmax)-inf(e_L_nextmax)), (sup(e_theta_nextmax)-inf(e_theta_nextmax))], 'EdgeColor', color_selector(2), 'LineWidth', 1)
hold on
e_theta_nextval = V*sin(delta_intmin)*Ts/L + e_theta
e_L_nextval = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*Ts/L)+delta_intmin))/sin(delta_intmin))
plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(2))
hold on
[t, x] = ode45(@sistema_dinamico, tspan, [e_L; e_theta], options, V, L, delta_intmin);
plot(x(:,1),x(:,2),'Color',color_selector(2))
hold on
xlabel('Lateral error (e_{L})')
ylabel('Yaw error (e_{\theta})')
hold off
end
%%
% data
V=1;
L=0.264;
lane_width=0.6;
delta_max=pi/6;
delta_min=-delta_max;
alpha=pi/6;
theta_fwsat=pi/3;
Ts=0.1;
incdelt=(delta_max-delta_min)/21;
% intervals
delta_int=infsup(delta_min,delta_max);
t_int=infsup(eps,Ts);
 % t_int=Ts;
eL_int=infsup(-lane_width/2,lane_width/2);

% intial states
e_theta_negs = [theta_fwsat, -theta_fwsat];
for j=1:length(e_theta_negs)
% e_theta=1.0472; % thetafw
e_theta=e_theta_negs(j);
% e_theta=0;
% e_L=0.229129; %vertice thetafw con upper ultimate safe trajectory
e_L=-0.2; %thetafw
% e_L=0;
% e_L = 0.262763;
% e_theta = 0.901408;
% interval projection
% Starting on deltamin
% Plot safe indicators state space
figure

load("eL0_fwsat.mat")
load("thetamax_fwsat.mat")
load("thetamin_fwsat.mat")
plot(eL0_fwsat,thetamin_fwsat,'k','LineWidth',1)
hold on
plot(eL0_fwsat,thetamax_fwsat,'k','LineWidth',1)
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'k','LineWidth',1)
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'k','LineWidth',1)
hold on

tspan = [0, Ts];  % Intervalo de tiempo
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

delta_intmin=delta_min
% interval projection
% Starting on deltamin
e_theta_nextmax = V*sin(delta_intmin)*t_int/L + e_theta
e_L_nextmax = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*t_int/L)+delta_intmin))/sin(delta_intmin))
% Plot safe indicators state space
plot(e_L,e_theta,'ok')
rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), (sup(e_L_nextmax)-inf(e_L_nextmax)), (sup(e_theta_nextmax)-inf(e_theta_nextmax))], 'EdgeColor', color_selector(1), 'LineWidth', 1)
hold on
e_theta_nextval = V*sin(delta_intmin)*Ts/L + e_theta
e_L_nextval = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*Ts/L)+delta_intmin))/sin(delta_intmin))
plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(1))
hold on
[t, x] = ode45(@sistema_dinamico, tspan, [e_L; e_theta], options, V, L, delta_intmin);
plot(x(:,1),x(:,2),'Color',color_selector(1))
hold on

delta_intmin=-e_theta;
e_theta_nextmax = V*sin(delta_intmin)*t_int/L + e_theta
e_L_nextmax = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*t_int/L)+delta_intmin))/sin(delta_intmin))
% Plot safe indicators state space
plot(e_L,e_theta,'ok')
rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), (sup(e_L_nextmax)-inf(e_L_nextmax)), (sup(e_theta_nextmax)-inf(e_theta_nextmax))], 'EdgeColor', color_selector(2), 'LineWidth', 1)
hold on
e_theta_nextval = V*sin(delta_intmin)*Ts/L + e_theta
e_L_nextval = e_L + (L*(cos(delta_intmin+e_theta)-cos(e_theta+(V*sin(delta_intmin)*Ts/L)+delta_intmin))/sin(delta_intmin))
plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(2))
hold on
[t, x] = ode45(@sistema_dinamico, tspan, [e_L; e_theta], options, V, L, delta_intmin);
plot(x(:,1),x(:,2),'Color',color_selector(2))
hold on
xlabel('Lateral error (e_{L})')
ylabel('Yaw error (e_{\theta})')
hold off
end
%%
% data
V=1;
L=0.264;
lane_width=0.6;
delta_max=pi/6;
delta_min=-delta_max;
alpha=pi/6;
theta_fwsat=pi/3;
Ts=0.1;
incdelt=(delta_max-delta_min)/21;
% intervals
delta_int=infsup(delta_min,delta_max);
t_int=infsup(eps,Ts);
 % t_int=Ts;
eL_int=infsup(-lane_width/2,lane_width/2);

% intial states
% e_theta_negs = [-pi/6, -0.5, -0.4, -0.3, -0.2, -0.1];
e_theta_negs = [pi/6, 0.5, 0.4, 0.3, 0.2, 0.1];
% e_theta_negs = [theta_fwsat, 1, 0.7, 0.6];
e_L_negs=[-0.3, -0.28, -0.26, -0.24, -0.22, -0.22];
for j=1:length(e_theta_negs)
% e_theta=1.0472; % thetafw
e_theta=e_theta_negs(j);
% e_theta=0;
% e_L=0.229129; %vertice thetafw con upper ultimate safe trajectory
% e_L=-0.3; %thetafw
% e_L=0;
% e_L = 0.262763;
% e_theta = 0.901408;
% interval projection
% Starting on deltamin
% Plot safe indicators state space
figure

load("eL0_fwsat.mat")
load("thetamax_fwsat.mat")
load("thetamin_fwsat.mat")
plot(eL0_fwsat,thetamin_fwsat,'k','LineWidth',1)
hold on
plot(eL0_fwsat,thetamax_fwsat,'k','LineWidth',1)
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)],[thetamin_fwsat(1) thetamax_fwsat(1)],'k','LineWidth',1)
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)],[thetamin_fwsat(end) thetamax_fwsat(end)],'k','LineWidth',1)
hold on

tspan = [0, Ts];  % Intervalo de tiempo
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
    for k=1:length(e_L_negs)
e_L=e_L_negs(k)
delta_intmax=delta_max
% interval projection
% Starting on deltamin
e_theta_nextmax = V*sin(delta_intmax)*t_int/L + e_theta
e_L_nextmax = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*t_int/L)+delta_intmax))/sin(delta_intmax))
% Plot safe indicators state space
plot(e_L,e_theta,'ok')
rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), (sup(e_L_nextmax)-inf(e_L_nextmax)), (sup(e_theta_nextmax)-inf(e_theta_nextmax))], 'EdgeColor', color_selector(1), 'LineWidth', 1)
hold on
e_theta_nextval = V*sin(delta_intmax)*Ts/L + e_theta
e_L_nextval = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*Ts/L)+delta_intmax))/sin(delta_intmax))
plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(1))
hold on
[t, x] = ode45(@sistema_dinamico, tspan, [e_L; e_theta], options, V, L, delta_intmax);
plot(x(:,1),x(:,2),'Color',color_selector(1))
hold on

A = 2*L/(V*Ts); % Sustituye con el valor deseado
B = e_theta; % Sustituye con el valor deseado

% Definir la función que queremos resolver
func = @(x) ((e_L_negs(1)-e_L)/L)*sin(x) + cos(x+e_theta) - cos(e_theta+(V*sin(x)*Ts/L)+x);

% Estimación inicial para x
x0 = 0;

% Usar fzero para encontrar una raíz de la función 
% delta_intmax=-e_theta;
delta_intmax= fzero(func, x0);
e_theta_nextmax = V*sin(delta_intmax)*t_int/L + e_theta
e_L_nextmax = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*t_int/L)+delta_intmax))/sin(delta_intmax))
% Plot safe indicators state space
plot(e_L,e_theta,'ok')
rectangle('Position', [inf(e_L_nextmax), inf(e_theta_nextmax), (sup(e_L_nextmax)-inf(e_L_nextmax)), (sup(e_theta_nextmax)-inf(e_theta_nextmax))], 'EdgeColor', color_selector(2), 'LineWidth', 1)
hold on
e_theta_nextval = V*sin(delta_intmax)*Ts/L + e_theta
e_L_nextval = e_L + (L*(cos(delta_intmax+e_theta)-cos(e_theta+(V*sin(delta_intmax)*Ts/L)+delta_intmax))/sin(delta_intmax))
plot(e_L_nextval,e_theta_nextval,'*','Color',color_selector(2))
hold on
[t, x] = ode45(@sistema_dinamico, tspan, [e_L; e_theta], options, V, L, delta_intmax);
plot(x(:,1),x(:,2),'Color',color_selector(2))
hold on
xlabel('Lateral error (e_{L})')
ylabel('Yaw error (e_{\theta})')
hold on
    end
    hold off
end
clear all
clc
V=1;
L=0.264;
lane_width=0.6;
delta_max=pi/6;
delta_min=-delta_max;
alpha=pi/6;
theta_fwsat=pi/3;
Ts=0.1;

delta=linspace(delta_min,delta_max,1000001);
e_theta0=pi/3-0.001;
 e_L0=0.21;

dgrid_vec=zeros([1 length(delta)]);
for p=1:length(delta)
    e_theta_next = V*sin(delta(p))*Ts/L + e_theta0;
    e_L_next = e_L0 + (L*(cos(delta(p)+e_theta0)-cos(e_theta0+(V*sin(delta(p))*Ts/L)+delta(p)))/sin(delta(p)));
    dgrid_vec(p) = isInvariantSet_fwsat(e_L_next,e_theta_next,theta_fwsat);
end
jmax=find(dgrid_vec,1,'last');
jmin=find(dgrid_vec,1,'first');
dgrid=[delta(jmin),delta(jmax)]

delta_int=infsup(dgrid(1),dgrid(2))
% e_L0=0.229129; %vertice thetafw con upper ultimate safe trajectory
t=infsup(eps,Ts);
detheta_ddelta=V*cos(delta_int)*t/L
deL_ddelta=(V*t*cos((V*sin(delta_int)*t + 3*L*(delta_int + e_theta0/3))/L) - V*t*cos((-V*sin(delta_int)*t + L*(-e_theta0 + delta_int))/L) + 4*L*(cos(e_theta0) - cos((V*sin(delta_int)*t + e_theta0*L)/L)))./(-2 + 2*cos(2*delta_int))
% deL_ddelta=((V.*cos(delta).*t+L).*sin((V*sin(delta)*t + L*(delta + e_theta0))/L) - L*sin(delta + e_theta0) + e_L0*cos(delta))./sin(delta) - (-L*cos((V*sin(delta)*t + L*(delta + e_theta0))/L) + L*cos(delta + e_theta0) + e_L0*sin(delta)).*cos(delta)./sin(delta).^2


%%
clear all
clc
V=1;
L=0.264;
lane_width=0.6;
delta_max=pi/6;
delta_min=-delta_max;
alpha=pi/6;
theta_fwsat=pi/3;
Ts=0.1;

delta=linspace(delta_min,delta_max,10001);
e_theta0=pi/3;
e_L0=0.2;

dgrid_vec=zeros([1 length(delta)]);
for p=1:length(delta)
    e_theta_next = V*sin(delta(p))*Ts/L + e_theta0;
    e_L_next = e_L0 + (L*(cos(delta(p)+e_theta0)-cos(e_theta0+(V*sin(delta(p))*Ts/L)+delta(p)))/sin(delta(p)));
    dgrid_vec(p) = isInvariantSet_fwsat(e_L_next,e_theta_next,theta_fwsat);
end
jmax=find(dgrid_vec,1,'last');
jmin=find(dgrid_vec,1,'first');
dgrid=[delta(jmin),delta(jmax)]

delta_int=linspace(dgrid(1),dgrid(2),1001)
% e_L0=0.229129; %vertice thetafw con upper ultimate safe trajectory
% t=infsup(eps,Ts);
t=Ts;
detheta_ddelta=V*cos(delta_int)*t/L;
deL_ddelta=(V*t*cos((V*sin(delta_int)*t + 3*L*(delta_int + e_theta0/3))/L) - V*t*cos((-V*sin(delta_int)*t + L*(-e_theta0 + delta_int))/L) + 4*L*(cos(e_theta0) - cos((V*sin(delta_int)*t + e_theta0*L)/L)))./(-2 + 2*cos(2*delta_int))
% deL_ddelta=((V.*cos(delta).*t+L).*sin((V*sin(delta)*t + L*(delta + e_theta0))/L) - L*sin(delta + e_theta0) + e_L0*cos(delta))./sin(delta) - (-L*cos((V*sin(delta)*t + L*(delta + e_theta0))/L) + L*cos(delta + e_theta0) + e_L0*sin(delta)).*cos(delta)./sin(delta).^2


% figure
% subplot(2,1,1)
% plot(delta_int,detheta_ddelta);
% hold on
% xline(delta(jmin),'g')
% hold on
% xline(delta(jmax),'g')
% hold on
% yline(0,'--')
% subplot(2,1,2)
% plot(delta_int,deL_ddelta);
% hold on
% xline(delta(jmin),'g')
% hold on
% xline(delta(jmax),'g')
% hold on
% yline(0,'--')

figure
% Primer subplot
subplot(2,1,1)
% Ajusta el color del plot usando la opción 'Color'
plot(delta_int, detheta_ddelta, 'Color', color_selector(3), 'LineWidth', 1);
% Agrega etiquetas a los ejes
xlabel('$\delta$ (rad)', 'Interpreter', 'latex');
ylabel('$\frac{d}{d \delta}e_{\theta}$', 'Interpreter', 'latex');
% Línea horizontal en y=0, ajusta el color usando 'Color'
yline(0, '--', 'Color', [0.5 0.5 0.5]);
xlim([min(delta_int) max(delta_int)]);

% Segundo subplot
subplot(2,1,2)
% Ajusta el color del plot usando la opción 'Color'
plot(delta_int, deL_ddelta, 'Color', color_selector(3), 'LineWidth', 1);
% Agrega etiquetas a los ejes
xlabel('$\delta$ (rad)', 'Interpreter', 'latex');
ylabel('$\frac{d}{d \delta}e_{L}$', 'Interpreter', 'latex');
% Línea horizontal en y=0, ajusta el color usando 'Color'
yline(0, '--', 'Color', [0.5 0.5 0.5]);

% Ajustes generales del gráfico
% Configurar límites del eje x si es necesario
xlim([min(delta_int) max(delta_int)]);
sgtitle('Study of the monotonicity of $e$ w.r.t. $\mathcal{D}$ for $e_{L_{0}} = 0.2$ m, $e_{\theta_{0}} = \theta_{fw}$', 'Interpreter', 'latex');

t_int=infsup(0,Ts);
delta_int=infsup(dgrid(1),dgrid(2));

e_theta_next1 = V*sin(dgrid(1))*t_int/L + e_theta0
e_L_next1 = e_L0 + (L*(cos(dgrid(1)+e_theta0)-cos(e_theta0+(V*sin(dgrid(1))*t_int/L)+dgrid(1)))/sin(dgrid(1)))
e_theta_next2 = V*sin(dgrid(2))*t_int/L + e_theta0
e_L_next2 = e_L0 + (L*(cos(dgrid(2)+e_theta0)-cos(e_theta0+(V*sin(dgrid(2))*t_int/L)+dgrid(2)))/sin(dgrid(2)))

tspan = [0, Ts];  % Intervalo de tiempo
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

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

plot(e_L0,e_theta0,'ok')
rectangle('Position', [inf(e_L_next1), inf(e_theta_next1), (sup(e_L_next1)-inf(e_L_next1)), (sup(e_theta_next1)-inf(e_theta_next1))], 'EdgeColor', color_selector(6), 'LineWidth', 1)
hold on
rectangle('Position', [inf(e_L_next2), inf(e_theta_next2), (sup(e_L_next2)-inf(e_L_next2)), (sup(e_theta_next2)-inf(e_theta_next2))], 'EdgeColor', color_selector(7), 'LineWidth', 1)


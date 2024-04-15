figure
grass=[0.24705882352941178,0.40784313725490196,0.10980392156862745];
sky=[0.21568627450980393,0.3686274509803922,0.592156862745098];
shadow=[0.16470588235294117,0.19215686274509805,0.19607843137254902];
sunset=[0.984313725490196,0.396078431372549,0.25882352941176473];

% S i
load("eL0_bicycle_draw.mat");
load("theta0max_bicycle_draw.mat")
load("theta0min_bicycle_draw.mat")
plot(eL0_bicycle_draw,theta0min_bicycle_draw,'Color',grass,'LineWidth',1.5,'DisplayName','S')
hold on
plot(eL0_bicycle_draw,theta0max_bicycle_draw,'Color',grass,'LineWidth',1.5,'HandleVisibility','off')
hold on
plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)], [theta0min_bicycle_draw(1) theta0max_bicycle_draw(1)],'Color',grass,'LineWidth',1.5,'HandleVisibility','off')
hold on
plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)], [theta0min_bicycle_draw(end) theta0max_bicycle_draw(end)],'Color',grass,'LineWidth',1.5,'HandleVisibility','off')
hold on
% Sfw i
load("eL0_fwsat.mat")
load("thetamax_fwsat.mat")
load("thetamin_fwsat.mat")
plot(eL0_fwsat,thetamin_fwsat,'Color',sky,'LineWidth',1.5,'DisplayName','S_{fw}')
hold on
plot(eL0_fwsat,thetamax_fwsat,'Color',sky,'LineWidth',1.5,'HandleVisibility','off')
hold on
plot([eL0_fwsat(1) eL0_fwsat(1)], [thetamin_fwsat(1) thetamax_fwsat(1)],'Color',sky,'LineWidth',1.5,'HandleVisibility','off')
hold on
plot([eL0_fwsat(end) eL0_fwsat(end)], [thetamin_fwsat(end) thetamax_fwsat(end)],'Color',sky,'LineWidth',1.5,'HandleVisibility','off')
hold on
% C i
% load("theta_fw_alpha_draw.mat")
% plot(eL0_bicycle_draw,theta_fw_alpha(:,1),'Color',sunset,'LineWidth',1.5,'DisplayName','^{i} C_{i+1}')
% hold on
% plot(eL0_bicycle_draw,theta_fw_alpha(:,2),'Color',sunset,'LineWidth',1.5,'HandleVisibility','off')
% hold on
% plot([eL0_bicycle_draw(1) eL0_bicycle_draw(1)], [theta_fw_alpha(1,1) theta_fw_alpha(1,2)],'Color',sunset,'LineWidth',1.5,'HandleVisibility','off')
% hold on
% plot([eL0_bicycle_draw(end) eL0_bicycle_draw(end)], [theta_fw_alpha(end,1) theta_fw_alpha(end,2)],'Color',sunset,'LineWidth',1.5,'HandleVisibility','off')
hold off
axis([min(eL0_bicycle_draw)*1.1 max(eL0_bicycle_draw)*1.1 min(theta0min_bicycle_draw)*1.1 max(theta0max_bicycle_draw)*1.1])
% Añadir leyenda
legend
xlabel('Lateral error (e_{L})')
ylabel('Yaw error (e_{\theta})')

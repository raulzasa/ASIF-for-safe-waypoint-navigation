function etheta_intersect=etheta_bounds_fw_alpha(eL0,theta_fw_sat,alpha,eL0_next)
Ts=0.05;
%V=0.37175;
V=1;
L=0.264;
width=0.6;
delta_max=pi/6;
delta_min=-delta_max;
eL_max=width/2;
eL_min=-eL_max;
% From changing lane orientation
if (0<=alpha)
    %alpha +
    etheta_max_next=acos((sin(delta_min).*(eL_max-eL0_next)./L)+1)-delta_min;
    etheta_min_next=-acos((sin(delta_max).*(eL_min-eL0_next)./L)+1)-delta_max;
else
    %alpha -
    etheta_max_next=acos((sin(delta_min).*(eL_max-eL0_next)./L)+1)-delta_min;
    etheta_min_next=-acos((sin(delta_max).*(eL_min-eL0_next)./L)+1)-delta_max;
end
etheta_nextSfw=intervalIntersection([etheta_min_next,etheta_max_next],[-theta_fw_sat,theta_fw_sat]);
etheta_changeOrientation_max=etheta_nextSfw(2)+alpha;
etheta_changeOrientation_min=etheta_nextSfw(1)+alpha;
etheta_changeOrientation=[etheta_changeOrientation_min,etheta_changeOrientation_max];
% From controlled invariant set
etheta_invSet_max=acos((sin(delta_min).*(eL_max-eL0)./L)+1)-delta_min;
etheta_invSet_min=-acos((sin(delta_max).*(eL_min-eL0)./L)+1)-delta_max;
etheta_invSet=[etheta_invSet_min,etheta_invSet_max];
% Forward oriented bounded
etheta_invSetfw=intervalIntersection(etheta_invSet,[-theta_fw_sat,theta_fw_sat]);
% Intersection
etheta_intersect=intervalIntersection(etheta_invSetfw,etheta_changeOrientation);
end
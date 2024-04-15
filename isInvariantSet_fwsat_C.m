function inv_out=isInvariantSet_fwsat_C(eL0,theta0,theta_fw_sat,alpha)
% Ts=0.05;
% V=0.37175;
% L=0.264;
% delta_max=pi/6;
% delta_min=-delta_max;
width=0.6;
eL_max=width/2;
eL_min=-eL_max;
% From changing lane orientation
if (0<=alpha)
    %alpha +
    etheta_intersect=etheta_bounds_fw_alpha(eL0,theta_fw_sat,alpha,eL_max);
else
    %alpha -
    etheta_intersect=etheta_bounds_fw_alpha(eL0,theta_fw_sat,alpha,eL_min);
end
% Returning binary decision
if ((eL_min*1.01<=eL0) && (eL0<=eL_max*1.01) && (etheta_intersect(1)*1.01<=theta0) && (theta0<=etheta_intersect(2)*1.01))
    inv_out = 1;
else
    inv_out = 0;
end
function [inv_out] = isInvariantSet_fwsat(eL0,etheta0,theta_fw_sat)
% This function returns a binary output determining if the state point
% belongs to the invariant set. The function considers the bicycle model

% DATA
%V=0.37175; %Linear velocity (cte)
V=1;
L=0.264; %Distance between front and rear wheels
width=0.6;
eL_max=width/2;
eL_min=-width/2;
delta_min=-pi/6;
delta_max=pi/6;
% Orientation boundaries
etheta_traj_max=acos((sin(delta_min).*(eL_max-eL0)./L)+1)-delta_min;
etheta_traj_min=-acos((sin(delta_max).*(eL_min-eL0)./L)+1)-delta_max;
% ANTI IMAGINARY
if (imag(etheta_traj_min)==0) || (imag(etheta_traj_max)==0)
    etheta_interval=intervalIntersection([etheta_traj_min,etheta_traj_max],[-theta_fw_sat,theta_fw_sat]);
    % Returning binary decision
    if ((eL_min<=eL0) && (eL0<=eL_max) && (etheta_interval(1)<=etheta0) && (etheta0<=etheta_interval(2)))
        inv_out = 1;
    else
        inv_out = 0;
    end
else
    inv_out = 0;
end
end
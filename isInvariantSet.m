function [inv_out] = isInvariantSet(eL0,phi0,model)
% This function returns a binary output determining if the state point
% belongs to the invariant set. The function considers the unicycle model 
% and the bicycle model (CM in rear wheels)

% DATA
V=0.37175; %Linear velocity (cte)
L=0.264; %Distance between front and rear wheels
eL_max=0.3; 
eL_min=-0.3;

% Model selection
if(model=="unicycle")
    omega_min=-2;
    omega_max=2;
    % Orientation boundaries
    phi_max=acos(((eL_max-eL0)*omega_min/V)+1);
    phi_min=-acos(((eL_min-eL0)*omega_max/V)+1);

    % Returning binary decision
    if ((eL_min<=eL0) && (eL0<=eL_max) && (phi_min<=phi0) && (phi0<=phi_max))
        inv_out = 1;
    else
        inv_out = 0;
    end
elseif(model=="bicycle_rear")
    delta_min=-pi/6;
    delta_max=pi/6;
    % Orientation boundaries
    phi_max=acos(((eL_max-eL0)*tan(delta_min)/L)+1);
    phi_min=-acos(((eL_min-eL0)*tan(delta_max)/L)+1);

    % Returning binary decision
    if ((eL_min<=eL0) && (eL0<=eL_max) && (phi_min<=phi0) && (phi0<=phi_max))
        inv_out = 1;
    else
        inv_out = 0;
    end
elseif(model=="bicycle_front")
    delta_min=-pi/6;
    delta_max=pi/6;
    % Orientation boundaries
    phi_max=acos((sin(delta_min).*(eL_max-eL0)./L)+1)-delta_min;
    phi_min=-acos((sin(delta_max).*(eL_min-eL0)./L)+1)-delta_max;

    % Returning binary decision
    if ((eL_min<=eL0) && (eL0<=eL_max) && (phi_min<=phi0) && (phi0<=phi_max))
        inv_out = 1;
    else
        inv_out = 0;
    end
end
end

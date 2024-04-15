function [x_next,y_next,theta_next] = nextBicycleStates(x,y,theta,delta,Ts)
%V=0.37175;
V=1;
L=0.264;
theta_next = V*sin(delta)*Ts/L + theta;
if abs(delta)<=1e-6
    x_next = x + V*Ts*cos(theta);
    y_next = y + V*Ts*sin(theta);
else
    x_next = x + (L*(-sin(delta+theta)+sin(theta+(V*sin(delta)*Ts/L)+delta))/sin(delta));
    y_next = y + (L*(cos(delta+theta)-cos(theta+(V*sin(delta)*Ts/L)+delta))/sin(delta));
end
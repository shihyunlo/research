function [ u_p, position_err_integral ] = ComputePositionControl( state_r, state, position_err_integral ) %linear

kp = [4.5 4.5 45]'*2;
kd = [4.5 4.5 20]'*1;
ki = [0.001 0.001 0.001]'*0;
position_err_integral = position_err_integral+state(1:3)-state_r(1:3);

u_p = [ -kp -kd ones(3,1) -ki].*[ state(1:3)-state_r(1:3) state(4:6)-state_r(4:6) state_r(7:9) position_err_integral];
u_p = sum(u_p,2);


end


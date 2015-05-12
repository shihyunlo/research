function [ u_a ] = ComputeAttitudeControl_2( u_p, stateA, yaw_d, ang_vel_d )

k_p = [316 316 500]'*10;
k_d = [323 323 500]'*1;

 phi_d = -u_p(2)/9.8;
 theta_d =u_p(1)/9.8;%%

u_a = k_p.*([phi_d theta_d yaw_d]' -stateA(1:3))+ k_d.*(ang_vel_d -stateA(4:6));
end




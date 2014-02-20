function [ u_a ] = ComputeAttitudeControl( u_p, quaternion )

%parameter to tune: first order time constant tau
tau = 0.1; % [0.08, 0.15] with 0.08 not directly angular-velocity-controllable, 0.1 controllable, 0.2 overshoot

if norm(u_p) ~=0
    u_p = u_p/norm(u_p);
end
zAxis_b = RotateVectByQuat([0 0 1]', quaternion);
alpha = acos(zAxis_b'*u_p);
k = cross(zAxis_b, u_p);
if norm(k) ~=0
    k = k/norm(k);
end
q_errRed = [cos(alpha/2); k*sin(alpha/2)];
q_cmdRed = QuatMultiply(quaternion, q_errRed);
q_1 = [quaternion(1); -quaternion(2:4)]/norm([quaternion(1); -quaternion(2:4)]);
q_e = QuatMultiply(q_1, q_cmdRed);
u_a = 2/tau*sign(q_e(1))*q_e(2:4);
end




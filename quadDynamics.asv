function state_dot = quadDynamics( state, u_p, u_a ) % assume force command exerted on current attitude

state_dot = zeros(10,1);
state_dot(1:3) = state(4:6);
zAxis_b = RotateVectByQuat([0 0 1]', state(7:10));
state_dot(4:6) = u_p'*zAxis_b*zAxis_b-9.8*[0 0 1]';
state_dot(7:10) = 0.5*QuatMultiply(state(7:10), [0; u_a]);

end


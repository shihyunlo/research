function state_dot = quadDynamics_3( state, u, J ) % quadDynamics without motor dynamics, u = acceleration; torque

state_dot = zeros(13,1);
state_dot(11:13) = J\(u(2:4)-cross(state(11:13),J*state(11:13)));
state_dot(7:10) = 0.5*QuatMultiply(state(7:10), [0; state(11:13)]);
R = QuatToR(state(7:10));
state_dot(1:3) = state(4:6);
state_dot(4:6) = u(1)*R*[0 0 1]'+9.8*[0 0 -1]';




end


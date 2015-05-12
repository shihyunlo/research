function state_dot = quadDynamics_2( state, w_d, MotorGain, kF, kM, L, J, Mass,D ) % assume force exerted on current attitude
g = 9.81;
state_dot = zeros(16,1);
state_dot(13:16) = MotorGain*(w_d-state(13:16));%motor model
% state(13:16) = w_d;%%
u = [kF kF kF kF; 0 kF*L 0 -kF*L; -kF*L 0 kF*L 0; kM -kM kM -kM]*[state(13)*state(13); state(14)*state(14); state(15)*state(15); state(16)*state(16)];
u = u+D;
R = ZYXToR(state(7),state(8),state(9));
state_dot(1:3) = state(4:6);
state_dot(4:6) = u(1)/Mass*R*[0 0 1]'+g*[0 0 -1]';
% H = [1 0 -sin(state(8)); 0 cos(state(7)) cos(state(8))*sin(state(7));  0  -sin(state(7)) cos(state(8))*cos(state(7))];
% state_dot(7:9) = H\state(10:12);
G = [1 tan(state(8))*sin(state(7)) cos(state(7))*tan(state(8)); 0 cos(state(7)) -sin(state(7));  0  sin(state(7))/cos(state(8)) cos(state(7))/cos(state(8))];
state_dot(7:9) = G*state(10:12);
state_dot(10:12) = J\(u(2:4)-cross(state(10:12),J*state(10:12)));


end


function state_dot = quadDynamics_PA( state, w_d, MotorGain, CT, CQ, L, J, Mass ) % assume force exerted on current attitude

state_dot = zeros(16,1);
state_dot(13:16) = MotorGain*(w_d-state(13:16));%motor model
% state(13:16) = w_d;%%
% normW = 7018;
% for j = 13:16
%     state(j) = min([state(j) normW]);
% end
u = [CT'; 0 CT(2)*L 0 -CT(4)*L; -CT(1)*L 0 CT(3)*L 0; CQ -CQ CQ -CQ]*[state(13)*state(13); state(14)*state(14); state(15)*state(15); state(16)*state(16)];

R = ZYXToR(state(7:9));
state_dot(1:3) = state(4:6);
state_dot(4:6) = u(1)/Mass*R*[0 0 1]'+9.8*[0 0 -1]';
H = [1 0 -sin(state(8)); 0 cos(state(7)) cos(state(8))*sin(state(7));  0  -sin(state(7)) cos(state(8))*cos(state(7))];
state_dot(7:9) = H\state(10:12);
state_dot(10:12) = J\(u(2:4)-cross(state(10:12),J*state(10:12)));





end


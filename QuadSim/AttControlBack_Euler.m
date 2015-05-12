function u_att = AttControlBack_Euler(state, Am2, inertia, w_d, w_dot_d)
G = [1 tan(state(8))*sin(state(7)) cos(state(7))*tan(state(8)); 0 cos(state(7)) -sin(state(7));  0  sin(state(7))/cos(state(8)) cos(state(7))/cos(state(8))];
w = state(10:12);
H = inertia\eye(3)*cross(w, inertia*w);
w_err = w-w_d;
err_ang_dot = G*w_err;
u_att = -inertia*(G'*err_ang_dot+H-w_dot_d+Am2*w_err);
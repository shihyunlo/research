function u_att = AttControl_Euler(u_pos, state, Kp_att, Kd_att, initial_state)

gravity = 9.81;
u_r = (u_pos(1)*sin(state(9)) - u_pos(2)*cos(state(9)))/gravity;
u_p = (u_pos(1)*cos(state(9)) + u_pos(2)*sin(state(9)))/gravity;
u_y = initial_state(9);
e_att = shortest_angular_distance([u_r;u_p;u_y], state(7:9));
e_ang = state(10:12);
u_att = -Kp_att*e_att - Kd_att*e_ang;

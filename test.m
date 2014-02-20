function state = test(state)  


p_d = 0;
    phi_d = 3;
    wPhi = kp_phi*(phi_d-state(1))+kd_phi*(p_d-state(2));
    w2 = wh+wPhi;
    w4 = wh-wPhi;
    [~,state_new] = ode45(@(t, state)rollDynamics(state, w2,w4,L,kF,Ixx) , [0 0.005], state); 
    state = state_new(end,:)';
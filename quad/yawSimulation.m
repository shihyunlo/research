
Y = [];
wF = 0;
wPhi = 0;
wTheta = 0;
state = [0 0]';
M = 0.853;
g = 9.8;
J = diag([2.32e-3, 2.32e-3 4.41e-3])*3.598;
Ixx = J(1,1);
Iyy = J(2,2);
Izz = J(3,3);
L = 0.175;
kF =  9.45432224e-08;
kM = 0.45*L*kF;
wh = sqrt(M*g/(4*kF));
del = 8*kM*wh/Izz;
% Q==1000; R == 1
  kp_psi = 22.3607*2;
  kd_psi = 42.0355*2;
 %poles at 1+-i
%   kp_psi = 56.6613*2;
%   kd_psi = 56.6613*2;


for t = 0 : 0.01: 10
    r_d = 0;
    psi_d = 1;
    wPsi = kp_psi*(psi_d-state(1))+kd_psi*(r_d-state(2));
    w1 = wh+wPsi;
    w2 = wh-wPsi;
    w3 = wh+wPsi;
    w4 = wh-wPsi;
    [~,state_new] = ode45(@(t, state)yawDynamics(state, w1,w2,w3,w4,kM,Izz) , [0 0.01], state); 
    state = state_new(end,:)';
    Y = [Y state];
end

plot(0:0.01:10, Y(1,:))
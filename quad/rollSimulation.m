
X = [];
state = [0 0]';
M = 0.853;
g = 9.8;
J = diag([2.32e-3, 2.32e-3 4.41e-3]);
Ixx = J(1,1);
Iyy = J(2,2);
Izz = J(3,3);
L = 0.175;
kF =  9.45432224e-08;
kM = 0.45*L*kF;
wh = sqrt(M*g/(4*kF));

del = 4*kF*L*wh/Ixx;

%Q == 1; R == 1
% kp_phi = 0.7071*2;
% kd_phi = 4.4125*2;
%Q==100; R == 1
%  kp_phi = 7.0711*2;
%  kd_phi = 15.4822*2;
% Q==500; R == 1
%  kp_phi = 15.8114*2;
%  kd_phi = 25.9649*2;
% Q==1000; R == 1
%   kp_phi = 70.71*2*1;
%   kd_phi = 83*2;
  % Q==1000000; R == 1
  kp_phi = 1000*20;
  kd_phi = 1007.4;
  
  kp_phi = 316.2*10;
  kd_phi = 323.5;
%   % Q==10000000; R == 1
%    kp_phi = 2236.1*135;
%    kd_phi = 2243.5;
%   
  
 %poles at -1,

 
  

for t = 0 : 0.005: 1
    p_d = 0;
    phi_d = 3;
    wPhi = kp_phi*(phi_d-state(1))+kd_phi*(p_d-state(2));
    w2 = wh+wPhi;
    w4 = wh-wPhi;
    [~,state_new] = ode45(@(t, state)rollDynamics(state, w2,w4,L,kF,Ixx) , [0 0.005], state); 
    state = state_new(end,:)';
    X = [X state];
end

plot(0:0.005:1, X(1,:))



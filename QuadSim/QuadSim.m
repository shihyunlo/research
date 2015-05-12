%% MATLAB code for quadrotor full system adaptive control performance simulation

%% vehicle parameter
m = 1.041;
gravity = 9.81;
L = 0.1793;
inertia = diag([0.009321095772;0.009400756981;0.01546128448]);
K = inertia\eye(3);
motor_const = 36;%27;
CT = 6.2738e-08;
CQ = 0.0263*CT;
wh = sqrt(m*gravity/4/CT);
M = [CT CT CT CT; 0 CT*L 0 -CT*L; -CT*L 0 CT*L 0; CQ -CQ CQ -CQ]\eye(4);
%% position control
Kp_pos = diag([5.3 5.3 14.3]);
Kd_pos = diag([3.7 3.7 6]);
PositionControl = @(ref, state) (Kp_pos*(ref(1:3)-state(1:3))+Kd_pos*(ref(4:6)-state(4:6))+ref(7:9));
%% attitude control-linearized euler angle
wn.rp = 30;
wn.yaw = 2.5;
xi.rp = 1;
xi.yaw = 1;
Kp_att = inertia*diag([wn.rp^2 wn.rp^2 wn.yaw^2]);
Kd_att = inertia*diag([2*wn.rp*xi.rp 2*wn.rp*xi.rp 2*wn.yaw*xi.yaw]);
%% attitude control-nonlinear quaternion
tau1 = 0.065;
tau2 = 0.01;
tau1 = 0.15;
tau2 = 0.04;
%% attitude controller-backstepping
fac = 0.4;
r_rp = 1/tau2;%not significant
p_rp = 125;
Am2 = diag(-[r_rp r_rp r_rp*fac]);
Asp = diag(-[p_rp p_rp p_rp*fac]);
S = [];
SS = [];
O_p = [];
%% simulated system parameters
att_rate = 200;
pos_rate = 100;
pos_command_delay = 0.08;%0.019; %bee
%% adaptive control parameter-L1 adaptive control
D = [0.0 0 0 0]';%disturbance

%% control loop simulation
dt = 1/att_rate; %attitude control rate, state feedback rate
prediction_rate = 4;%state prediction rate per attitude control loop
dT = dt/prediction_rate;
T = 4;
initial_state = [0 0 0 0 0 0 0 0 0 0 0 0 wh wh wh wh]';
max_rpm = 8500;
min_rpm = 1000;
state = initial_state;
w_d = ones(4,1)*wh;
X = [];
Xn = [];
Xf = [];
U = [];
TT = [];
U_POS = [];
U_T = [];
U_QUAT = [];
U_attQ = [];
U_attB = [];
pos_command_time = [];
u_pos = [0 0 0]';
uT = m*gravity;
%%disturbance observation
sigma = zeros(3,1);
ss = zeros(3,1);
nu_a = zeros(3,1);
nu_b = zeros(3,1);
omega_p = zeros(3,1);
omega_f = zeros(3,1);
i = 1;
%% motor control response time
tau_m = 0.1;
PHI = ((eye(3)-exp(Asp*dt)))\eye(3);
delta = 0.4;
gamma = 0.6;%noise0.2?;
alpha = 0.97;%no-noise, motor dynamics, time-delay(helping)0.96;%0.9-ideal 
beta = 0.7;%needed with noise, to compensate for noisy prediction
nu = zeros(3,1);
NU = [];
NU_A = [];
HH = [];
for t = 0:dt:T
  

    state_ref = [0 0 0 0 0 0 0 0 0]';

    
    %position control
    if mod(t/dt,floor(att_rate/pos_rate)) == 0
        U_POS(:,i) = PositionControl(state_ref, state);
        U_T(i) = m*(U_POS(3,i) + gravity);
        pos_command_time(i) = t;
        i = i+1;
    end
    %time delay
    pos_sent_time = t - pos_command_delay;
    if pos_sent_time>=0
        pos_command_idx = find(pos_sent_time>=pos_command_time,1,'last');
        u_pos = U_POS(:,pos_command_idx);
        uT = U_T(pos_command_idx);
        uT(uT<0) = 0;
    end
    
    if t>1
        D = [0 0.2 0 0]';
    end
    
    %attitude control
    %euler linear
    u_att = AttControl_Euler(u_pos, state, Kp_att, Kd_att, initial_state);
    %quaternion
    u_att_quat = AttControlQuat(u_pos, ...
        ZYXToQuat(state(7), state(8), state(9)), tau1);%desired angular velocity
    U_QUAT = [U_QUAT u_att_quat];
    omega = state(10:12);
    H = cross(omega,inertia*omega);
    u_attQ = inertia*(u_att_quat-omega)/tau2 + H;
    U_attQ = [U_attQ u_attQ];
    
    %euler backstepping
    omega_dot_d = (u_att_quat-omega)/tau2;
    u_att_back = AttControlBack_Euler(state, Am2, inertia, u_att_quat, omega_dot_d);
    U_attB = [U_attB u_att_back];
    
    %disturbance observation    
    nu_b = u_attQ;%baseline control
    omega_f = delta*omega_f + (1-delta)*omega;
    e = omega_p-omega;%prediction error
    e = omega_p-omega_f;%prediction error on filtered dynamics
    H_ = -K*cross(omega_f,inertia*omega_f);
    omega_p_dot = H_ + K*(nu+sigma);
    omega_p = omega_p + omega_p_dot*dt;%*0.62;%%tuned vector
    O_p = [O_p omega_p];
%     sigma = -inertia*Asp*PHI*exp(Asp*dt)*e;
    sigma_dot = -gamma*e-gamma*omega_p_dot;    
    sigma = sigma+sigma_dot*dt;
%     ss = beta*ss + (1-beta)*sigma;
    
    
    G = [1 tan(state(8))*sin(state(7)) cos(state(7))*tan(state(8));...
        0 cos(state(7)) -sin(state(7));  0  sin(state(7))/cos(state(8)) cos(state(7))/cos(state(8))];
    nu_a = alpha*nu_a + (1-alpha)*-(sigma);%sigma%-inertia*G'*(G*(-u_att_quat+omega)));
    NU_A = [NU_A nu_a];
    S = [S sigma];
    SS = [SS ss];
    nu = nu_b+nu_a;
    NU = [NU nu];
    HH = [HH H];
    
    %desired motor rpm
    ws_d = M*[uT;nu];
    %desire angular velocity from backstepping considering motor dynamics
    w_dot_d = (sqrt(ws_d)-w_d)/tau_m;
    w_d = sqrt(ws_d);
    w_d = w_d + w_dot_d/motor_const;
    w_d(w_d>max_rpm) = max_rpm;
    w_d(w_d<min_rpm) = min_rpm;
    [~,state_new] = ode45(@(t, state)quadDynamics_2(state, w_d, motor_const, CT, CQ, L, inertia, m, D) , [0 dt], state); 
    state = real(state_new(end,:)');
    X = [X state];
    state(1:6) = state(1:6) + randn(6,1)*0.003;%system noise
    state(7:9) = state(7:9) + randn(3,1)*pi/180*0.5;
    state(10:12) = state(10:12) + randn(3,1)*0.002;
    Xn = [Xn state];
    Xf = [Xf omega_f];
    U = [U [u_pos; u_att;w_d]];
    
end
figure
plot(0:dt:T,X(10,:));hold on;
plot(0:dt:T,Xf(1,:),'r')
xlabel('time(s)')
ylabel('angular velocity  (rad/s)')
legend('measured anular velocity','predicted angular velocity','location','SouthEast')
title('state prediction for piecewise update law')

figure
r = zeros(size(0:dt:T));
r(ceil(length(r)/4):end) = 0.2;
plot(0:dt:T,r,'r',0:dt:T,S(1,:),'b',0:dt:T,-NU_A(1,:),'m');
legend('reference disturbance','observed disturbance','adaptive input','location','SouthEast')
% plot(0:dt:T,r,'r',0:dt:T,S(1,:),'b',0:dt:T,SS(1,:),'r',0:dt:T,-NU_A(1,:),'m');
% legend('reference disturbance','observed disturbance','filtered observation','adaptive input','location','SouthEast')
title('transient response convergence and disturbance observation')
% 
figure
plot(0:dt:T,U_QUAT(1,:)); hold on; plot(0:dt:T,X(10,:),'m')
xlabel('time(s)')
ylabel('angular velocity  (rad/s)')
legend('desired agular velocity','measured angular velocity','location','SouthEast')
title('adapted attitude controller tracking performance w/ motor dynamics')
% 
figure
plot(0:dt:T,X(2,:),0:dt:T,X(3,:))
xlabel('time(s)')
ylabel('position  (m)')
legend('y response','z response','location','SouthEast')
ylim([-0.4 0.1])


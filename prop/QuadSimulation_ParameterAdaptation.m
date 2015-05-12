clear all
close all
% implementation and simulation of second order attitude control
%%

% state = [position velocity euler_ang ang_vel motor_ang_vel]';[16x1]
% state_dot = [velocity aceeleration euler_ang_dot angular_accel
% motor_ang_accel]';[16x1]
%WaypointsConstraints = [ConstraintsX ; ConstraintsY; ConstraintsZ]';[19x3]
%waypoints constructed xyz-and-yaw-independently

%% initualize
Mass = 1.03;
J = diag([9.3e-4, 9.4e-3 1.5e-2])*1;
L = 0.175;
MotorGain = 11;
kF = 6.1726e-008;
kM = 0.45*L*kF;
%simulation parameters
T = 10;
%% vx<4 vy<4 vz<2
 TrajSeg = 4;
  WaypointsConstraints =  [0 0 0 0 0 40 65 70 100 0 0 0 0;
                           0 0 0 0 0 35 70 85 100 0 0 0 0;
                           0 10 0 0 0 20 40 60 100 -10 0 0 0]'/20;


X = [];
U = [];
W = [];
 Traj = GenerateTrajectory( T,TrajSeg, WaypointsConstraints); %Traj == waypoints position; velocity; acceleration
yaw_desired = 0;
ang_vel_desired = [0 0 0]';
position_err_integral = [0 0 0]';
w_h = sqrt(Mass*9.8/(4*kF));

state = [0 0 0 0 0 0 0 0 0 0 0 0 w_h w_h w_h w_h]';
state_dot = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ]';
s = 1;
dt = 0.01;

%% GP training cov
cov_t = [0.5 2.58 0.2];
cov_t_vw = [0.5 2.56 0.2];
cov_s = [0.1 1.05 0.2];
scaling = 100;
[vW, normV, ~, y_CTt, y_P] = propGPtraining_vW(cov_t_vw, scaling);
[PW, normP, normW, y_CTs, y_CQs, y_vs] = propGPtraining(cov_s, scaling);
%% coefficients initialization
vt = [0 0 0 0]';
kt1 = covSEard(cov_t_vw, scaling*vW, scaling*[vt/normV state(13:end)/normW]);
CTt = kt1'*y_CTt + 6.1889e-008;
P = kt1'*y_P;
CQt = P./w_h^3*60/2/pi;
CT = CTt*0.9;
CTs = CT;
CQs = CQt;
vs = [0 0 0 0]';
w_h = sqrt(Mass*9.8/sum(CT));
CTrs = CTt;
CTs_ = [];
CQs_ = [];
Vs_ = [];
CTt_ = [];
Vt_ = [];
CQt_ = [];
w = w_h*ones(4,1);
%% System Dynamics Simulation
alpha = 0.5;
% alpha = 1;
alpha2 = 0.02;
alpha2 = 1;
state_reference = [0 0 0 0 0 0 0 0 0]';
thr = 0.4;
for t = 0 : dt: T
%      state_reference = Traj(:,s);
    % simulated paramter compute
%     Pnoise = P+randn(4,1);
%     Wnoise = w+randn(4,1);
%     ks = covSEard(cov_s, scaling*PW, scaling*[P/normP w/normW]);
%     new_CTs = ks'*y_CTs + CTrs;
%  
%         CTs = (1-alpha)*CTs + alpha*new_CTs;
%         CQs = (1-alpha)*CQs + alpha*ks'*y_CQs;
%         vs = (1-alpha2)*vs + alpha2*ks'*y_vs;
%         noise compensation
    for i = 1:4
        Pnoise = [P(i) P(i)*1.1 P(i)*0.9]';
        w_ = [w(i) w(i) w(i)]';
        ks = covSEard(cov_s, scaling*PW, scaling*[Pnoise/normP w_/normW]);
        new_CTs(i,1) = sum(ks'*y_CTs.*[0.5 0.25 0.25]') + CTrs(i);
        new_vs(i,1) = sum(ks'*y_vs.*[0.5 0.25 0.25]');
    end
    

        CTs = (1-alpha)*CTs + alpha*new_CTs;
%         CQs = (1-alpha)*CQs + alpha*ks'*y_CQs;
        vs = (1-alpha2)*vs + alpha2*new_vs;

    CTs_ = [CTs_ CTs];
    CQs_ = [CQs_ CQs];
    Vs_ = [Vs_ vs];
    
    [u_p, position_err_integral] = ComputePositionControl(state_reference, state(1:6), position_err_integral); %force
    w_f = u_p(3)*Mass/(2*sum(CT)*w_h);
    u_a = ComputeAttitudeControl_2(u_p, state(7:12), yaw_desired, ang_vel_desired);
    ratio = sqrt(CTs./CT);
    r = 1./ratio;
%     r = ones(1,4);
    w_d = [r(1) 0 -r(1) 1; r(2) -r(2) 0 -1; r(3) 0 r(3) 1; r(4) -r(4) 0 -1]*[ w_h+w_f; u_a];
    [~,state_new] = ode45(@(t, state)quadDynamics_PA(state, w_d, MotorGain, CTt, kM, L, J, Mass) , [0 dt], state); 
    state = state_new(end,:)';
    for j = 13:16
        state(j) = min([state(j) normW*0.97]);
        state(j) = max([state(j) 4050]);
    end
    w = [state(13); state(14); state(15); state(16)];
    X = [X state];
    W = [W w];
    
    % real paramter compute
    if t > 5
        vt = [2 0 0 0]';
    end
    kt1 = covSEard(cov_t_vw, scaling*vW, scaling*[vt/normV w/normW]);
    CTt = kt1'*y_CTt + CTrs;
    P = kt1'*y_P;
    Vt_ = [Vt_ vt];
    CTt_ = [CTt_ CTt];
    CQt = P./w.^3*60/2/pi;
    CQt_ = [CQt_ CQt];
    
    s = s+1;
   
end

%% plots

plot(0:dt:T ,X(3,:))
title('z position tracking error')
xlabel('time(s)')
ylabel('tracking error(m)')

figure
plot(0:dt:T ,Vs_(1,:))
hold on
plot(0:dt:T ,Vt_(1,:), 'r')
title('wind speed prediction')
xlabel('time(s)')
ylabel('wind speed(m/s)')
legend('wind speed prediction', 'true wind speed')
mv = max(max(Vs_(1,:)), max(Vt_(1,:)));
axis([0 10 min(Vt_(1,:))-0.1*mv mv*1.5])

figure
plot(0:dt:T ,CTs_(1,:))
hold on
plot(0:dt:T ,CTt_(1,:), 'r')
title('CT adaptation')
xlabel('time(s)')
ylabel('CT value(N/(rpm^2))')
legend('Adapted CT values', 'true CT values')
mvv = max(max(CTs_(1,:)), max(CTt_(1,:)));
axis([0 10 min(Vt_(1,:))-0.1*mvv mvv*1.5])
% plot(0:0.01:T ,X(8,:))
% figure(3)
% plot3(Traj(1,:), Traj(2,:), Traj(3,:));
% hold on
% plot3(X(1,:), X(2,:), X(3,:),'r');
% % plot(0.2*ones(size(X(7,:))),'black');
% % legend('reference roll angle','roll angle','pitch angle','yaw angle');
% % hold on
% figure(1)
% plot(0:0.01:T, X(1,:),'r');
% 
% hold on
% plot(0:0.01:T, X(2,:),'g');
% 
% hold on
% plot(0:0.01:T, X(3,:))
% legend('x', 'y', 'z');
% 
% figure(2)
% plot(0:0.01:T, X(4,:),'r');
% 
% hold on
% plot(0:0.01:T, X(5,:),'g');
% 
% hold on
% plot(0:0.01:T, X(6,:))
% legend('v_x', 'v_y', 'v_z');
% plot(dd, 'y')
% hold on

% plot(X(7,:),'r');
% 
% hold on
% plot(X(8,:),'g');
% 
% hold on
% plot(X(9,:))





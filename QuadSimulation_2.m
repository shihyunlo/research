clear all
close all
% implementation and simulation of second order attitude control
%%

% state = [position velocity euler_ang ang_vel motor_ang_vel]';[16x1]
% state_dot = [velocity aceeleration euler_ang_dot angular_accel
% motor_ang_accel]';[16x1]
%WaypointsConstraints = [ConstraintsX ; ConstraintsY; ConstraintsZ]';[19x3]
%waypoints constructed xyz-and-yaw-independently

%initualize

%Quad Properties from Vishnu
Mass = 0.853;
J = diag([2.32e-3, 2.32e-3 4.41e-3])*1;
L = 0.175;
MotorGain = 11;
kF =  9.45432224e-08;
kM = 0.45*L*kF;
%simulation parameters
T = 10;
%% vx<4 vy<4 vz<2
 TrajSeg = 4;
  WaypointsConstraints =  [0 0 0 0 0 40 65 70 100 0 0 0 0;
                           0 0 0 0 0 35 70 85 100 0 0 0 0;
                           0 10 0 0 0 20 40 60 100 -10 0 0 0]'/20;


X = [];

 Traj = GenerateTrajectory( T,TrajSeg, WaypointsConstraints); %Traj == waypoints position; velocity; acceleration
yaw_desired = 0;
ang_vel_desired = [0 0 0]';
position_err_integral = [0 0 0]';
w_h = sqrt(Mass*9.8/(4*kF));

state = [0 0 0 0 0 0 0 0 0 0 0 0 w_h w_h w_h w_h]';
state_dot = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ]';
s = 1;
%% x<2 y<1 z<0.5 due to higher z gain, z<1
for t = 0 : 0.01: T
     state_reference = Traj(:,s);
%     state_reference = [2 1 0.5 0 0 0 0 0 0]';
    [u_p, position_err_integral] = ComputePositionControl(state_reference, state(1:6), position_err_integral); %force
    w_f = u_p(3)*Mass/(8*kF*w_h);
    u_a = ComputeAttitudeControl_2(u_p, state(7:12), yaw_desired, ang_vel_desired);
    w_d = [1 0 -1 1; 1  1 0 -1; 1 0 1 1; 1 -1 0 -1]*[ w_h+w_f; u_a];
    [~,state_new] = ode45(@(t, state)quadDynamics_2(state, w_d, MotorGain, kF, kM, L, J, Mass) , [0 0.01], state); 
    state = state_new(end,:)';
    X = [X state];
    
    s = s+1;
   
end
figure(3)
plot3(Traj(1,:), Traj(2,:), Traj(3,:));
hold on
plot3(X(1,:), X(2,:), X(3,:),'r');
% plot(0.2*ones(size(X(7,:))),'black');
% legend('reference roll angle','roll angle','pitch angle','yaw angle');
% hold on
figure(1)
plot(0:0.01:T, X(1,:),'r');

hold on
plot(0:0.01:T, X(2,:),'g');

hold on
plot(0:0.01:T, X(3,:))
legend('x', 'y', 'z');

figure(2)
plot(0:0.01:T, X(4,:),'r');

hold on
plot(0:0.01:T, X(5,:),'g');

hold on
plot(0:0.01:T, X(6,:))
legend('v_x', 'v_y', 'v_z');
% plot(dd, 'y')
% hold on

% plot(X(7,:),'r');
% 
% hold on
% plot(X(8,:),'g');
% 
% hold on
% plot(X(9,:))





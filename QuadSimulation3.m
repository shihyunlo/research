% implementation and simulation of nonlinear attitude controller of ETHZ techinical report 2013
%%
%system modeled as first order system with body angular velocity as direct
%control input
% X = [position velocity euler angles]'; [9x1]
% state = [position velocity quaternion angular velocity]';[13x1]
% state_dot = [velocity aceeleration quaternion_dot angular acceleration]'
%WaypointsConstraints = [ConstraintsX ; ConstraintsY; ConstraintsZ]';[19x3]
%waypoints constructed xyz-independently

%initualize
state = [0 0 0 0 0 10 1 0 0 0 0 0 0]';
state_dot = [0 0 10 0 0 0 0 0 0 0 0 0 0]';
state = [0 0 0 0 0 0 1 0 0 0 0 0 0]';
state_dot = [0 0 0 0 0 0 0 0 0 0 0 0 0]';
J = diag([2.32e-3, 2.32e-3 4.41e-3])*1;
T = 10;
TrajSeg = 4;
WaypointsConstraints =  [0 0 0 0 0 40 65 70 100 0 0 0 0;
                         0 0 0 0 0 35 70 85 100 0 0 0 0;
                         0 10 0 0 0 20 40 60 100 -10 0 0 0]';
X = [];
test = [];

Traj = GenerateTrajectory( T,TrajSeg, WaypointsConstraints); %Traj == waypoints position; velocity; acceleration
position_err_integral = [0 0 0]';
err_integral = [0 0 0]';
s = 1;
for t = 0 : 0.01: T
    state_reference = Traj(:,s);
    state_reference = [0.1 0 0.1 0 0 0 0 0 0]';
    [u_p, position_err_integral] = ComputePositionControl(state_reference, state(1:6), position_err_integral); %acceleration
    if u_p>14
        u_p = 14;
    end
    u_a = ComputeAttitudeControl(u_p, state(7:10)); %angular velocity
    u_a = u_a/4;
    [Torque, err_integral] = OnbaordControl(u_a, state(11:13), err_integral, J);%Torque
    %torque to motor speed, fit the quadratic motor model
%     wm = sqrt([kF kF kF kF; 0 kF*L 0 -kF*L; -kF*L 0 kF*L 0; kM -kM kM -kM]\[u_p; Torque])
test = [test [u_p; u_a; Torque]];

    [~,state_new] = ode45(@(t, state)quadDynamics_3( state, [u_p; Torque], J) , [0 0.01], state); 
    state = state_new(end,:)';
    X_new = [state(1:6) ;QuatToZYX(state(7:10)); state(11:13)];
    X = [X X_new];
    s = s+1;
end

plot3(Traj(1,:), Traj(2,:), Traj(3,:));
hold on
plot3(X(1,:), X(2,:), X(3,:),'r');




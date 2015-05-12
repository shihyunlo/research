% implementation and simulation of nonlinear attitude controller of ETHZ techinical report 2013
%%
%system modeled as first order system with body angular velocity as direct
%control input
% X = [position velocity euler angles]'; [9x1]
% state = [position velocity quaternion ]';[10x1]
% state_dot = [velocity aceeleration quaternion_dot]'
%WaypointsConstraints = [ConstraintsX ; ConstraintsY; ConstraintsZ]';[19x3]
%waypoints constructed xyz-independently

%initualize
state = [0 0 0 0 0 10 1 0 0 0 ]';
state_dot = [0 0 10 0 0 0 0 0 0 0 ]';
T = 10;
TrajSeg = 4;
WaypointsConstraints =  [0 0 0 0 0 40 65 70 100 0 0 0 0;
                         0 0 0 0 0 35 70 85 100 0 0 0 0;
                         0 10 0 0 0 20 40 60 100 -10 0 0 0]';
X = [];

Traj = GenerateTrajectory( T,TrajSeg, WaypointsConstraints); %Traj == waypoints position; velocity; acceleration
position_err_integral = [0 0 0]';
s = 1;
for t = 0 : 0.01: T
    state_reference = Traj(:,s);
    [u_p, position_err_integral] = ComputePositionControl(state_reference, state(1:6), position_err_integral); %acceleration
    u_a = ComputeAttitudeControl(u_p, state(7:10)); %angular velocity
    [~,state_new] = ode45(@(t, state)quadDynamics(state, u_p, u_a) , [0 0.01], state); 
    state = state_new(end,:)';
    X_new = [state(1:6) ;QuatToZYX(state(7:10))];
    X = [X X_new];
    s = s+1;
end

plot3(Traj(1,:), Traj(2,:), Traj(3,:));
hold on
plot3(X(1,:), X(2,:), X(3,:),'r');




% test ETHZ onboard PI attitude controller
% takes in euler angle, test for the body angular velocity tracking
% performance
%% response time:~0.0015, within 2.5pi rad/s; ~0.004, with 6pi rad/s
J = diag([2.32e-3, 2.32e-3 4.41e-3])*1;
angVel = [0 0 0]';
err_integral = [0 0 0]';
A = [];
T = 0.01;

for t = 0:0.0005:T
    angVel_d = [3*pi 0 0]';
    [Torque, err_integral] = OnbaordControl(angVel_d, angVel, err_integral, J);
    [~,angVel_new] = ode45(@(t, angVel)quadAngularDynamics(angVel,Torque, J), [0 0.01], angVel); 
    angVel = angVel_new(end,:)';
    A = [A angVel];
end

plot(0:0.0005:T, A(1,:),'r');

hold on
% plot(0:0.001:T, A(2,:),'g');
% 
% hold on
% plot(0:0.001:T, A(3,:))



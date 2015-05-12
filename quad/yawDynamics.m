function yaw_dot = yawDynamics(yaw, w1, w2, w3, w4, kM, Izz)

yaw_dot = zeros(2,1);
yaw_dot(1) = yaw(2);
yaw_dot(2) = kM*(w1^2-w2^2+w3^2-w4^2)/Izz;

end
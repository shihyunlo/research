function roll_dot = rollDynamics(roll, w2, w4, L, kF, Ixx)

roll_dot = zeros(2,1);
roll_dot(1) = roll(2);
roll_dot(2) = L*kF*(w2^2-w4^2)/Ixx;

end
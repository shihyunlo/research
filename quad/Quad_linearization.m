syms x y z v_x v_y v_z phi theta psi p q r M g L kF kM Ixx Iyy Izz w1 w2 w3 w4 kp_r1 kp_r3 kd_r1 kd_r3 kp_a1 kp_a3 kd_a1 kd_a3
kp_r = [kp_r1; kp_r1; kp_r3];
kd_r = [kd_r1; kd_r1; kd_r3];

%Quad system linearization with ZYX rotation

psi_d = 0;
p_d = 0;
q_d = 0;
r_d = 0;

%value assignment

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

%roll simulation
syms phi phi_dot kp_phi kd_phi
del = 4*kF*L*wh/Ixx;
A = [0 1; -kp_phi*del -kd_phi*del ];

%state/control input specification
X = [x; y; z; v_x; v_y; v_z; phi; theta; psi; p; q; r];
u = [w1; w2; w3; w4];

%governing equations linearization
R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta); 
    -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi) cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi) sin(phi)*cos(theta);
    sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi) -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi) cos(phi)*cos(theta)];
r_double_dot = 1/M*([0 0 1]'*M*g+ R*[0 0 1]'*kF*(w1^2+w2^2+w3^2+w4^2));
A_r = jacobian(r_double_dot, X);
B_r = jacobian(r_double_dot, u);
w_dot = [1/Ixx*(L*kF*(w2^2-w4^2)-q*r*(Izz-Iyy));
         1/Iyy*(L*kF*(w3^2)-w1^2)-p*r*(Ixx-Izz);
         1/Izz*(kM*(w1^2-w2^2+w3^2-w4^2))];
A_w = jacobian(w_dot, X);
B_w = jacobian(w_dot, u);
H = [1 0 -sin(theta); 0 cos(phi) sin(phi)*cos(theta); 0 -sin(phi) cos(phi)*cos(theta)];
A_p = sym(zeros(3,12));
A_p(:,4:6) = sym(eye(3));
A_a = sym(zeros(3,12));
A_a(:,10:12) = H\(eye(3));

%feedback control loop involved
r_ = [x; y; z];
r_dot = [v_x; v_y; v_z];
%r_double_dot_d = r_double_dot_T+kd.*(r_dot_d-r_dot)+kp.*(r_d-r); %ki term not shown but used in controller
r_double_dot_d = kd_r.*-r_dot+kp_r.*-r;
wF = r_double_dot_d(3)*M/(8*kF*wh);
phi_d = 1/g*(r_double_dot_d(2));
theta_d = -1/g*(r_double_dot_d(1));
wPhi = kp_a1*(phi_d-phi)+kd_a1*(p_d-p);
wTheta = kp_a1*(theta_d-theta)+kd_a1*(q_d-q);
wPsi = kp_a3*(psi_d-psi)+kd_a3*(r_d-r);

%u = [1 0 -1 1; 1 1 0 -1; 1 0 1 1; 1 -1 0 -1]*[wF; wPhi; wTheta; wPsi]; check if u is the same as manually decomposed
T(1,6) = -M*kd_r3/(8*kF*wh);
T(1,12) = -M*kp_r3/(8*kF*wh);
T(2,12) = -kp_a1*kp_r1/g;
T(2,5) = -kp_a1*kd_r1/g;
T(2,7) = -kp_a1;
T(2,10) = -kd_a1;
T(3,12) = kp_a1*kp_r1/g;
T(3,4) = kp_a1*kd_r1/g;
T(3,8) = -kp_a1;
T(3,11) = -kd_a1;
T(4,9) = -kp_a3;
T(4,12) = -kd_a3;
%T*X = [wF; wPhi; wTheta; wPsi]; therefore A' = A + B* [1 0 -1 1; 1 1 0 -1; 1 0 1 1; 1 -1 0 -1]*T

A = [A_p; A_r+B_r*[1 0 -1 1; 1 1 0 -1; 1 0 1 1; 1 -1 0 -1]*T; A_a; A_w+B_w*[1 0 -1 1; 1 1 0 -1; 1 0 1 1; 1 -1 0 -1]*T];





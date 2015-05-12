function R = QuatToR(q)

% qx = q.x;
% qy = q.y;
% qz = q.z;
% qw = q.w;

qx = q(2);
qy = q(3);
qz = q(4);
qw = q(1);
R = zeros(3, 3);
R(1, 1) = qw*qw + qx*qx - qy*qy - qz*qz;
R(1, 2) = 2*qx*qy - 2*qw*qz;
R(1, 3) = 2*qx*qz + 2*qw*qy;
R(2, 1) = 2*qx*qy + 2*qw*qz;
R(2, 2) = qw*qw - qx*qx + qy*qy - qz*qz;
R(2, 3) = 2*qy*qz - 2*qw*qx;
R(3, 1) = 2*qx*qz - 2*qw*qy;
R(3, 2) = 2*qy*qz + 2*qw*qx;
R(3, 3) = qw*qw - qx*qx - qy*qy + qz*qz;
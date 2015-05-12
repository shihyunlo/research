function anglevec = QuatToZYX(quat)
% Assumes input is q = [qx qy qz qw]
    a = quat(4); % w
    b = quat(1); % x
    c = quat(2); % y
    d = quat(3); % z
    
    rot = zeros(3,3);
    rot(1,1) = a*a + b*b - c*c - d*d;
    rot(1,2) = 2*b*c - 2*a*d;
    rot(1,3) = 2*b*d + 2*a*c;
    rot(2,1) = 2*b*c + 2*a*d;
    rot(2,2) = a*a - b*b + c*c - d*d;
    rot(2,3) = 2*c*d - 2*a*b;
    rot(3,1) = 2*b*d - 2*a*c;
    rot(3,2) = 2*c*d + 2*a*b;
    rot(3,3) = a*a - b*b - c*c + d*d;
    
theta = -asin(rot(3,1));
phi = 0;
psi = 0;

if cos(theta) > 1e-6
    phi = atan2(rot(3,2),rot(3,3));
    psi = atan2(rot(2,1),rot(1,1));
end

anglevec = [phi theta psi]';
    
end
function rot = QuatToR(quat)
% This is the MATLAB version of the QuatToR function in
% geometry/utils/src/GeometryUtils.cc
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

end


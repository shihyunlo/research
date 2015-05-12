function q = RToQuat(rot)

theta = -asin(rot(3,1));
phi = 0;
psi = 0;

if cos(theta) > 1e-6
    phi = atan2(rot(3,2),rot(3,3));
    psi = atan2(rot(2,1),rot(1,1));
end

q = zeros(4,1);
q(1) = sin(phi/2)*cos(theta/2)*cos(psi/2)-cos(phi/2)*sin(theta/2)*sin(psi/2);
q(2) = cos(phi/2)*sin(theta/2)*cos(psi/2)+sin(phi/2)*cos(theta/2)*sin(psi/2);
q(3) = cos(phi/2)*cos(theta/2)*sin(psi/2)-sin(phi/2)*sin(theta/2)*cos(psi/2);
q(4) = cos(phi/2)*cos(theta/2)*cos(psi/2)+sin(phi/2)*sin(theta/2)*sin(psi/2);


end
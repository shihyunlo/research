function q = ZYXToQuat(phi, theta, psi)

R = ZYXToR(phi, theta, psi);
q = RToQuat(R);
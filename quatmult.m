function f = quatmult(q,p)

% assumes q = [qx qy qz qw], p = [px py pz pw]

L = [q(4)*eye(3)-hat3(q(1:3)) [q(1); q(2); q(3)];
      -q(1) -q(2) -q(3)   q(4)];
  
f = L*p;

end
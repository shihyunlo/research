function [ vector_r ] = RotateVectByQuat( vector, q )

if size(vector,1) ==1
    vector = vector';
end

q_cg = [q(1); -q(2:4)];
q_vec = [0; vector];
vector_r = QuatMultiply(QuatMultiply(q,q_vec), q_cg);
vector_r = vector_r(2:4);


end


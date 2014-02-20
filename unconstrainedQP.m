function [solution] = unconstrainedQP(T, seg, Df)


%seg = 10;
%Df = [start_point_constraints mid_points_position_constraints end_point_constraints]';
%Df = [0 1 1 0 0 10 20 30 40 50 60 70 80 90 100 -1 -1 0 0]';

%T = 10;
t = T/seg;
poly_ord = 9;
k = poly_ord+1;

%tic
q = zeros(poly_ord+1, poly_ord+1);
Q = [];
A = [];
A_up = [diag([1 1 2 6 24]) zeros(5,5)];
A_down = [1 t t^2 t^3 t^4 t^5 t^6 t^7 t^8 t^9;
          0 1 2*t 3*t^2 4*t^3 5*t^4 6*t^5 7*t^6 8*t^7 9*t^8;
          0 0 2 6*t 12*t^2 20*t^3 30*t^4 42*t^5 56*t^6 72*t^7;
          0 0 0 6 24*t 60*t^2 120*t^3 210*t^4 336*t^5 504*t^6;
          0 0 0 0 24 120*t 360*t^2 840*t^3 1680*t^4 3024*t^5];
a = [A_up;A_down];  



for s = 1:seg
    for i = 5:k
        for j = 5:k
        
        q(i,j) = 2*(j-1)*(j-2)*(j-3)*(j-4)*(i-1)*(i-2)*(i-3)*(i-4)*t^(i-1+j-1-2*4+1)/(i-1+j-1-2*4+1);
        
        end
    end
    Q = blkdiag(Q,q);
    A = blkdiag(A,a);
end

M1 = zeros(k*s, k/2*seg+k/2);
M1(1:5, 1:5) = eye(5);
for s = 1:seg-1
    M1(s*10-4:s*10+5, 5*s+1:5*s+5) = [eye(5);eye(5)];
end
M1(10*seg-4:10*seg, 5*seg+1:5*seg+5) = eye(5);
M1 = M1';

M2 = zeros(k/2*seg+k/2, k/2*seg+k/2);
M2(1:5,1:5) = eye(5);
for i = 1:seg-1
    M2(i+5, i*5+1) = 1;
    M2(seg+6+4*i:seg+9+4*i, i*5+2:i*5+5) = eye(4);
end
M2(seg+5:seg+9, 5*seg+1:5*seg+5) = eye(5);

% %
% M2_new = zeros(k/2*seg+k/2, k/2*seg+k/2);
% for i = 1:seg-1
%     M2_new(i, (i-1)*5+1) = 1;
%     M2_new(seg+6+4*i:seg+9+4*i, i*5+2:i*5+5) = eye(4);
% end
% 
%  M2_new(seg, (seg-1)*5+1) = 1;
%  M2_new(seg+1, seg*5+1) = 1;
%  
% for j = 1:4
% M2_new(seg+2*j, j+1) = 1;
% M2_new(seg+2*j+1, 5*seg+1+j) = 1;
% end


%

% M2 = M2_new;
M = M2*M1;
A_ = inv(A);
R = M*A_'*Q*A_*M';

solution = [];
for n = 1:size(Df,2)
Dp_star = -inv(R(seg+10:5*seg+5,seg+10:5*seg+5))*R(1:seg+9,seg+10:5*seg+5)'*Df(:,n);
D = [Df(:,n);Dp_star];
d = M1'*M2'*D;
solution = [solution A_*d];
end


%toc


    



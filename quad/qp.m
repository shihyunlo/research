
seg = 10;
T = 10;
t = T/seg;
poly_ord = 9;
k = poly_ord+1;
q = zeros(poly_ord+1, poly_ord+1);
Q = [];
A = [];
A_up = [diag([1 1 2 6 24]) zeros(5,5)];
A_down = [1 t t^2 t^3 t^4 t^5 t^6 t^7 t^8 t^9;
          0 1 2*t 3*t^2 4*t^3 5*t^4 6*t^5 7*t^6 8*t^7 9*t^8;
          0 0 2 6*t 12*t^2 20*t^3 30*t^4 42*t^5 56*t^6 72*t^7;
          0 0 0 6 24*t 60*t^2 120*t^3 210*t^4 336*t^5 504*t^6;
          0 0 0 0 6 24 120*t^2 360*t^3 840*t^4 1680*t^5 3024*t^6];
a = [A_up;A_down];  

for s = 1:seg
    for i = 5:k
        for j = 5:k
        
        q(i,j) = 2*(j-1)*(j-2)*(j-3)*(j-4)*(i-1)*(i-2)*(i-3)*(i-4);
        
        end
    end
    Q = blkdiag(Q,q);
    A = blkdiag(A,a);
end

b = 
x = quadprg(Q,[],[],[],A,b);
        
        

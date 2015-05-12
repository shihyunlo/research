E = zeros(2,17);

for i = 1:17
    E(:,i) = svd(data([i i+17 i+17*2],1:2));
end
e = E(1,:)./(E(2,:));
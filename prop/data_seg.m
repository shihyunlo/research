E = zeros(2,17);

for i = 1:17
    E(:,i) = svd(data_s([i i+17 i+17*2],1:2));
end
e = E(1,:)./(E(2,:));
d = mean(e);
v = var(e);
d_w = 7013-5012;
d_p = d_w/d;

Ws = unique(data_s(:,2));
Ps = unique(data_s(:,1));



data1 = [data_s(1:11,:); data_s(1+17:11+17,:); data_s(1+17*2:11+17*2,:)];
data2 = [data_s(7:17,:); data_s(7+17:17+17,:); data_s(7+17*2:17+17*2,:)];



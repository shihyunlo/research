 


x = [];
y = [];
z = [];
vx = [];
vy = [];
vz = [];
ax = [];
ay = [];
az = [];
seg = 4;


for s = 1:seg
for t = 0.1:0.25:10/seg
time1 = [1 t t^2 t^3 t^4 t^5 t^6 t^7 t^8 t^9];
time2 = [0 1 2*t 3*t^2 4*t^3 5*t^4 6*t^5 7*t^6 8*t^7 9*t^8];
time3 = [0 0 2 6*t 12*t^2 20*t^3 30*t^4 42*t^5 56*t^6 72*t^7];
time4 = [0 0 0 6 24*t 60*t^2 120*t^3 210*t^4 336*t^5 504*t^6];
time5 = [0 0 0 0 24 120*t 360*t^2 840*t^3 1680*t^4 3024*t^5];
x1 = time1*solution1((s-1)*10+1:(s-1)*10+10);
y1 = time1*solution2((s-1)*10+1:(s-1)*10+10);
z1 = time1*solution3((s-1)*10+1:(s-1)*10+10);
vx1 = time2*solution1((s-1)*10+1:(s-1)*10+10);
vy1 = time2*solution2((s-1)*10+1:(s-1)*10+10);
vz1 = time2*solution3((s-1)*10+1:(s-1)*10+10);
ax1 = time3*solution1((s-1)*10+1:(s-1)*10+10);
ay1 = time3*solution2((s-1)*10+1:(s-1)*10+10);
az1 = time3*solution3((s-1)*10+1:(s-1)*10+10);

x = [x x1];
y = [y y1];
z = [z z1];
vx = [vx vx1];
vy = [vy vy1];
vz = [vz vz1];
ax = [ax ax1];
ay = [ay ay1];
az = [az az1];

end
end

plot3(x,y,z);
hold on
quiver3(x,y,z,vx,vy,vz);
hold on
quiver3(x,y,z,ax,ay,az,'r');

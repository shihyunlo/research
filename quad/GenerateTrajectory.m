function [ state_reference ] = GenerateTrajectory( FlightTime,TrajSeg, WaypointsConstraints)

poly_degree = 9;
solution = unconstrainedQP(FlightTime, TrajSeg, WaypointsConstraints,poly_degree);

x = [];
y = [];
z = [];
vx = [];
vy = [];
vz = [];
ax = [];
ay = [];
az = [];

for s = 0:TrajSeg-1
for t = 0.01:0.01:FlightTime/TrajSeg
time1 = [1 t t^2 t^3 t^4 t^5 t^6 t^7 t^8 t^9];
time2 = [0 1 2*t 3*t^2 4*t^3 5*t^4 6*t^5 7*t^6 8*t^7 9*t^8];
time3 = [0 0 2 6*t 12*t^2 20*t^3 30*t^4 42*t^5 56*t^6 72*t^7];
% time4 = [0 0 0 6 24*t 60*t^2 120*t^3 210*t^4 336*t^5 504*t^6];
% time5 = [0 0 0 0 24 120*t 360*t^2 840*t^3 1680*t^4 3024*t^5];
x1 = time1*solution(s*10+1:s*10+10,1);
y1 = time1*solution(s*10+1:s*10+10,2);
z1 = time1*solution(s*10+1:s*10+10,3);
vx1 = time2*solution(s*10+1:s*10+10,1);
vy1 = time2*solution(s*10+1:s*10+10,2);
vz1 = time2*solution(s*10+1:s*10+10,3);
ax1 = time3*solution(s*10+1:s*10+10,1);
ay1 = time3*solution(s*10+1:s*10+10,2);
az1 = time3*solution(s*10+1:s*10+10,3);

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


state_reference = [WaypointsConstraints(1,1) x; WaypointsConstraints(1,2) y; WaypointsConstraints(1,3) z; 
                   WaypointsConstraints(2,1) vx; WaypointsConstraints(2,2) vy; WaypointsConstraints(2,3) vz;
                   WaypointsConstraints(3,1) ax; WaypointsConstraints(3,2) ay; WaypointsConstraints(3,3) az];


end


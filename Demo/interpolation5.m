%% interpolation5
% This function implements the quintic polynomial interpolation
% 
% q0:       the position of the start point
% v0:       the velocity of the start point
% acc0:     the acceleration of the start point
% t0:       the current time of the start point
% q1:       the position of the end point
% v1:       the velocity of the end point
% acc1:     the acceleration of the end point
% t1:       the current time of the end point

%waypoints: the data of waypoints, which are calculated by the quintic polynomial interpolation



function waypoints = interpolation5(q0,v0,acc0,t0,q1,v1,acc1,t1)

accuracy = 0.001;

t = t1-t0;
d = q1-q0;

a0 = q0;
a1 = v0;
a2 = acc0/2;
a3 = ( 20*d - (8*v1 + 12*v0)*t - (3*acc0 - acc1)*t^2 ) / (2*t^3);
a4 = ( -30*d + (14*v1 + 16*v0)*t + (3*acc0 - 2*acc1)*t^2 ) / (2*t^4);
a5 = ( 12*d - (6*v1 + 6*v0)*t + (-acc0 + acc1)*t^2 ) / (2*t^5);

%%
p1 = [];
for x = 0 : accuracy : t1
    y1 = a0 + a1*x + a2*x^2 + a3*x^3 + a4*x^4 + a5*x^5;
    p1 = [p1,y1];
end

p2 = [];
for x = 0 : accuracy : t1
    y2 =  a1 + 2*a2*x + 3*a3*x^2 + 4*a4*x^3 + 5*a5*x^4;
    p2 = [p2,y2];
end

p3 = [];
for x = 0 : accuracy : t1
    y3 = 2*a2 + 6*a3*x + 12*a4*x^2 + 20*a5*x^3;
    p3 = [p3,y3];
end

waypoints = [p1;p2;p3];

end


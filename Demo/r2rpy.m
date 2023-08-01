%% r2rpy
% Calculate the Euler angle from Rotation matrix

function angle = r2rpy(R)

o = rotm2eul(R,'ZYX');
angle = [o(3); o(2); o(1)];


% angle = zeros(3,1);
% 
% % z (roll)
% angle(3) = atan2(R(2,1), R(1,1));
% % y (pitch)
% angle(2) = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
% % x (yaw)
% angle(1) = atan2(R(3,2), R(3,3));

end
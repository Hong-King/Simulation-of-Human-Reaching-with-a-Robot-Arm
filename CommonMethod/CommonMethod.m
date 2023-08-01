function [Q,waypoints,loopPoints] = CommonMethod(DH_params, jtype, q, pDes, oDes, runningTime)

accuracy = 0.001;
LOOPTIME = runningTime / accuracy;


%% calculate pIni,oIni
[TCur_k,~] = FK(DH_params, jtype, q);
pIni = TCur_k(1:3,4);
oIni = r2rpy(TCur_k(1:3,1:3));

%%
vector = [pDes; oDes] - [pIni; oIni];

distance = sqrt( vector(1)^2 + vector(2)^2 + vector(3)^2 );

waypoints = interpolation5(0,0,0,0,distance,0,0,runningTime);

loopPoints= [];
for i = 1 : LOOPTIME
    p = [pIni; oIni] + vector .* (waypoints(1,i+1) / distance);

    loopPoints = [loopPoints, p];
end

% plot3(loopPoints(1,:), loopPoints(2,:), loopPoints(3,:),"*-");
% hold on
% grid on
% plot3(pDes(1),pDes(2),pDes(3),'o',MarkerFaceColor='r');



Q = [];
thetaK = q;  % initial joint angle
for i = 1 : LOOPTIME
    QF = IK(DH_params, jtype, thetaK, loopPoints(1:3,i), loopPoints(4:6,i));
    Q = [Q,QF];
    thetaK = Q(:,end);
end

end



%% IKJ
% Pseudo-Inverse Matrix Solving
function JTranspose = IKJ(J)

[n,m] = size(J);
k = 0.01; % Avoid singular solutions

if n>m
    JTranspose = (J' * J + k^2 .* eye(m,m)) \ J';
else
    JTranspose = J' / (J  * J' + k^2 .* eye(n,n));
end

end

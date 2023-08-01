function [Q,waypoints,loopPoints] = CoRLEGO(DH_params, jtype, q, pDes, oDes, pItem1, pItem2, runningTime)

accuracy = 0.001;
LOOPTIME = runningTime / accuracy;


%% calculate pIni,oIni
[TCur_k,~] = FK(DH_params, jtype, q);
pIni = TCur_k(1:3,4);
oIni = r2rpy(TCur_k(1:3,1:3));

%% using outloop to get the waypoints
waypoints = [0;0;0];
loopPoints= [];
for i = 1 : LOOPTIME

    pFake = targetLocaion( pDes , pItem1 , pItem2, i);

    vector = [pFake; oDes] - [pIni; oIni];

    distance = sqrt( vector(1)^2 + vector(2)^2 + vector(3)^2 );

    w = interpolation5(0,0,0,0,distance,0,0,runningTime);
    waypoints = [waypoints, w(:,i+1)];

    p = [pIni; oIni] + vector .* (waypoints(1,i+1) / distance);
    loopPoints = [loopPoints, p];
end

%% only used in test, to check this function
% plot3(loopPoints(1,:), loopPoints(2,:), loopPoints(3,:),".-");
% hold on
% grid on
% plot3(pDes(1),pDes(2),pDes(3),'o',MarkerFaceColor='r');
% plot3(pItem1(1),pItem1(2),pItem1(3),'o',MarkerFaceColor='g');
% plot3(pItem2(1),pItem2(2),pItem2(3),'o',MarkerFaceColor='b');

%% using inverse kinematics to follow these waypoints
Q = [];
thetaK = q;  % initial joint angle
for i = 1 : LOOPTIME
    QF = IK(DH_params, jtype, thetaK, loopPoints(1:3,i), loopPoints(4:6,i));
    Q = [Q,QF];
    thetaK = Q(:,end);
end

end

%% Human Attention Shift
function pdes = targetLocaion( pdes1 , pdes2 , pdes3, time)

w = 1/(1+exp(-time / 100));
pdes = (pdes1.*w + pdes2.*(1-w) + pdes3.*(1-w))./(2-w);

end






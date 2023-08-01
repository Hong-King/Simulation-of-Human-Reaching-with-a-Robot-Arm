clear all;close all;clc;
%% TEST example
% This scripts is an example of how your code will be tested. sim_robot plot
% the robot in the 3D space given the DH parameters and the joint
% configuration. You can use the simulation for visual inspection purposes
% in both questions.

% DH parameters format based on Siciliano's book:
%  
%      | a_1 | alpha_1 | d_1 | theta_1 |
% DH = | ... | ....... | ... | ....... | 
%      | a_n | alpha_n | d_n | theta_n |
%
% with alpha_i and theta_i in radiant

%% Constant
DOF = 7;

%% Parameter
runningTime = 0.6;      % The running time of robot arm
accuracy = 0.001;       % Sampling interval (time)

%% 7-DOF Humanoid Robot Arm
q = [0 , pi/3 , 0 , pi/3 , 0 , 0, 0]';
jtype = [0; 0; 0; 0; 0; 0; 0];
DH(:,1) =[0.36, 0, 0.42, 0, 0.4, 0, 0.126];            % a  
DH(:,2) = [0, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2];           % alpha
DH(:,3) = [0, 0, 0, 0, 0, 0, 0];           % d
DH(:,4) = q;                    % theta


%% Generate target position and orientation (calculated by desired joint configuration)
% Demo1
q_des1 = [0.581126789590012;
0.866645034908981;
1.13074963775876;
1.96632473823060;
0.233990990294307;
4.25394944878051;
4.45220474165290];    % desired joint configuration

% Demo2
% q_des1 = [0.982351009815711;
% 2.66240350328917;
% 3.01739838890699;
% 1.96520263622332;
% 0.970636749623199;
% 4.46702959492359;
% 0.386751451934193];     % desired joint configuration

% Demo3
% angleLimit = 3*pi/2;      % joint angle limit
% q_des1 = rand(DOF,1) * angleLimit;        % (randomly generated) desired joint configuration 

[T,~] = FK(DH, jtype, q_des1);
p_des1 = T(1:3,4);               % desired end-effector position
R = T(1:3,1:3);
phi_des1 = r2rpy(R);          % desired end-effector orientation


%% call CommonMethod function
[Q1,waypoints1,loopPoints1] = CommonMethod(DH, jtype, q, p_des1, phi_des1, runningTime);


%% Show the results
% plot the joint angle profile
figure()
subplot(6,2,[1,3,5]);
plot(transpose(Q1))
iterations_1 = length(Q1(1,:))
xlabel('Iterations')
ylabel('Angle')
title('Joint parameters')
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7'); %'q_1','q_2','q_3','q_4','q_5','q_6','q_7'

% plot the displacement, velocity and acceleration profile
time = 0 : accuracy : runningTime;
subplot(6,2,7);
plot(time, waypoints1(1,:))
xlabel('Time (s)')
ylabel('Displacement (m)')
subplot(6,2,9);
plot(time, waypoints1(2,:))
xlabel('Time (s)')
ylabel('Velocity (m/s)')
subplot(6,2,11);
plot(time, waypoints1(3,:))
xlabel('Time (s)' )
ylabel('Acceleration (m/s2)')

% Plot the trajectory
P = [];
for i = 1:size(Q1,2)
[T,~] = FK(DH, jtype, Q1(:,i));
P(:,i) = T(1:3,4);
end
subplot(6,2,[2,4,6,8,10,12]);
plot3(P(1,:),P(2,:),P(3,:),'.-')
xlabel('x')
ylabel('y')
zlabel('z')
title('Tracks');
hold on

plot3(loopPoints1(1,:), loopPoints1(2,:), loopPoints1(3,:),".-");
grid on

plot3(p_des1(1),p_des1(2),p_des1(3),'o',MarkerFaceColor='r');

[TCur_k,~] = FK(DH, jtype, q);
pIni = TCur_k(1:3,4);
plot3(pIni(1),pIni(2),pIni(3),'o',MarkerFaceColor='y');
legend('CoRLEGO','Loop Points','Target','Initial Point'); % ,'Outer Loop'
hold off

%% Error
[T,~] = FK(DH, jtype, Q1(:,end));

pfinal = T(1:3,4);
R = T(1:3,1:3);
phifinal = r2rpy(R);

% Check the final error between the current end-effector pose and the desired one
p_error1 = p_des1-pfinal
o_error1 = phi_des1-phifinal


%% Show the simulated robotic arm
%Simulate the robot at each step of the iterative inverse kinematics and observe how the end-effector reaches the desired pose
% figure()
% sim_robot(DH,q,jtype)
% for steps = 1:size(Q1,2)
%     sim_robot(DH,Q1(:,steps),jtype)
% end
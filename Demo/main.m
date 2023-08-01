clear all;close all;clc;

%% Constant
DOF = 7; %The degree of freedom of Humanoid Robot Arm

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

%% Call inverse kinematics function
Q1 = IK(DH, jtype, q, p_des1, phi_des1);

%% Show results
figure()

% plot the joint angle profile
subplot(1,2,1);
plot(transpose(Q1))
grid on
iterations_1 = length(Q1(1,:))
xlabel('Num Iteration')
ylabel('Joint Angle')
title('Common Method')
legend('q_1','q_2','q_3','q_4','q_5','q_6'); %'q_1','q_2','q_3','q_4','q_5','q_6'

% Plot the trajectory
P = [];
for i = 1:size(Q1,2)
    [T,~] = FK(DH, jtype, Q1(:,i));
    P(:,i) = T(1:3,4);
end
subplot(1,2,2);
plot3(P(1,:),P(2,:),P(3,:),'.-')
xlabel('x')
ylabel('y')
zlabel('z')
title('Tracks');
hold on
grid on
plot3(p_des1(1),p_des1(2),p_des1(3),'o',MarkerFaceColor='r');
legend('Trajectory','Target'); % ,'Outer Loop'
title('Common Method')
hold off

%% Show the simulated robotic arm
% figure()
% sim_robot(DH,q,jtype)
% for steps = 1:size(Q1,2)
%     sim_robot(DH,Q1(:,steps),jtype)
% end

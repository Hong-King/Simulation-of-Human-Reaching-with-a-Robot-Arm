function Q = IK(DH_params, jtype, q, pdes, odes)
% IK calculates the inverse kinematics of a manipulator
% Q = IK(DH_params, jtype, q, pdes, odes) calculates the joint
% values of a manipulator robot given the desired end-effector's position
% and orientation by solving the inverse kinematics, iteratively.  The
% inputs are DH_params, jtype and q, pdes, and odes.  DH_params is
% nx4 matrix including Denavit-Hartenberg parameters of the robot.  jtype
% and q are n-dimensional vectors.  jtype describes the joint types of the
% robot and its values are either 0 for revolute or 1 for prismatic joints.
% q is the joint values.  pdes and odes are desired position and
% orientation of the end-effector.  pdes is 2x1 for a planar robot and 3x1
% for a spatial one.  odes is a scalar for a planar robot and a 3x1 vector
% for a 3D robot.  Orientation is defined as roll-pitch-yaw angles.  Use
% Jacobian Transpose to compute the inverse of the Jacobian.

%% iterative inverse kinematics
k = 0.01;
ERROR = 0.001;
time = 200;  

Q = q;  % joint angle
thetaK = q;  % initial joint angle
iTime = 1;
single = 1; % if value is 1, continue to iterate

% Calculate the oritention of the end effector in the current stage
[TCur_k,J_k] = FK(DH_params, jtype, q);

% Reference frame given the desired orientation in RPY angles
r_z = odes(3); p_y = odes(2); y_x = odes(1);

Rrpy_des = [cos(r_z)*cos(p_y), cos(r_z)*sin(p_y)*sin(y_x) - sin(r_z)*cos(y_x), cos(r_z)*sin(p_y)*cos(y_x) + sin(r_z)*sin(y_x);
    sin(r_z)*sin(p_y), sin(r_z)*sin(p_y)*sin(y_x) - cos(r_z)*cos(y_x), sin(r_z)*sin(p_y)*cos(y_x) + cos(r_z)*sin(y_x);
    -sin(p_y)        , cos(p_y)*sin(y_x)                             , cos(p_y)*cos(y_x)                            ];

% Approximate end-effector velocity, determine the direction of movement
ep = pdes - TCur_k(1:3,4);
eo = 1/2*(cross(TCur_k(1:3,1),Rrpy_des(:,1)) + cross(TCur_k(1:3,2),Rrpy_des(:,2)) + cross(TCur_k(1:3,3),Rrpy_des(:,3)) );
e = [ep; eo];

% calculate the error
if sum(abs(e),'all')<ERROR || iTime>= time
    single = 0; % stoping iterating
end

while single

    JTranspose = IKJ(J_k);

    %% Gradient Project Method (GPM)
    % Joint limit range
    qmax = deg2rad([170 120 170 120 170 120 175]);  % Convert angles from degrees to radians
    qmin = deg2rad([-170 -120 -170 -120 -170 -120 -175]);

    % K is zoom coefficient.
    % When we want minimize the objective function, it is negative;
    % When we want maximize the objective function, it is positive.
    K = -10;

    for j = 1:7
        a(j) = (qmax(j)+qmin(j))/2;   % Median value of the j-th joint

        % Gradient of optimization function
        gradient(j) = 2*(thetaK(j)-a(j))/(7*(a(j)-qmax(j))^2); 
    end

    %% GPM
    I = eye(7,7);

    % Limit Avoidance Index
    NJ = K*(I-JTranspose*J_k)*gradient';  % gradient is a 7×1 vector

    dO = JTranspose *  (e .* k) + NJ ; % Variation of joint angles
    thetaK = dO + thetaK;

    % Calculate the oritention of the end effector in the next stage
    [TCur_k,J_k] = FK(DH_params, jtype, thetaK);

    % Approximate end-effector velocity, determine the direction of movement
    ep = pdes - TCur_k(1:3,4);
    eo = 1/2*(cross(TCur_k(1:3,1),Rrpy_des(:,1)) + cross(TCur_k(1:3,2),Rrpy_des(:,2)) + cross(TCur_k(1:3,3),Rrpy_des(:,3)) );
    e = [ep; eo];

    % calculate the error
    if sum(abs(e),'all')<ERROR || iTime>= time
        single = 0; % stoping iterating
    end

    Q = [Q,thetaK];
    iTime = iTime +1;
end

end

%% IKJ
% Pseudo-Inverse Matrix Solving
% using Damped Least-Squares (DLS) inverse to avoid singular solutions
function JTranspose = IKJ(J)

[n,m] = size(J);
k = 0.001; % used to Avoid singular solutions

if n>m
    JTranspose = (J' * J + k^2 .* eye(m,m)) \ J';
else
    JTranspose = J' / (J  * J' + k^2 .* eye(n,n));
end

end
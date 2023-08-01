function [T, J] = FK(DH_params, jtype, q)
% FK calculates the forward kinematics of a manipulator
% [T, J] = FK(DH_params, jtype, q) calculates the Homogeneous
% transformation matrix from base to the end-effector (T) and also the
% end-effector Jacobian with respect to the base frame of a manipulator
% robot.  The inputs are DH_params, jtype and q.  DH_params is nx4 matrix
% including Denavit-Hartenberg parameters of the robot.  jtype and q are
% n-dimensional vectors.  jtype describes the joint types of the robot.
% Its values are either 0 for revolute or 1 for prismatic joints.  q is the
% vector of joint values.

% DH = [ a, alpha, d, theta]

n = size(q,1);  % robot's DoF
% consistency check
if (n~=size(DH_params,1)) || (n~=size(jtype,1))
    error('inconsistent in dimensions');
end

% initialisation
T = eye(4,4);
J = zeros(6,n);

P = zeros(3,n+1);
Z = zeros(3,n);
Z(3,1) = 1;

%% T homogeneous matrix calculations
for i = 1:n
    if jtype(i)==0  && DH_params(i,3) ~= 0 % revolute
        error('wrong value d of the ' + i +' joint, because this is a revolute joint');
    end

    A11 = cos(q(i));
    A12 = - sin(q(i)) * cos(DH_params(i,2));
    A13 = sin(q(i)) * sin(DH_params(i,2));
    A14 = DH_params(i,1) * cos(q(i));

    A21 = sin(q(i));
    A22 = cos(q(i)) * cos(DH_params(i,2));
    A23 = -cos(q(i)) * sin(DH_params(i,2));
    A24 = DH_params(i,1) * sin(q(i));
 
    A31 = 0;
    A32 = sin(DH_params(i,2));
    A33 = cos(DH_params(i,2));
    A34 = DH_params(i,3);

    A = [A11,A12, A13, A14;
        A21,A22, A23, A24;
        A31,A32, A33, A34;
        0, 0, 0, 1];

    T = T * A;
    P(:,i+1) = T(1:3,4);
    Z(:,i+1) = T(1:3,3);
end

%% J Jacobian calculations
for i = 1:n
    if jtype(i)==1  %prismatic
        J(1:3,i) = Z(:,i);
        J(4:6,i) = [0,0,0];
    else             % revolute
        J(1:3,i) = crossProduct(Z(:,i), (P(:,n+1)  - P(:,i)) );
        J(4:6,i) = Z(:,i);
    end
end
end

%% calculate cross product
function c = crossProduct(u,v)
c(1) = u(2) * v(3) - u(3) * v(2);
c(2) = u(3) * v(1) - u(1) * v(3);
c(3) = u(1) * v(2) - u(2) * v(1);
end

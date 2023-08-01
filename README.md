Author: Junhong Liu.
\\
Supervisor: Dr. Dietmar Heinke & Dr. Masoumeh Iran Mansouri
\\
Platform: Matlab 2022b.

# Simulation of Human Reaching with a Robot Arm

> Please note: If you want to visualize the robotic arm, the dependent [Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/) is necessary.

## 1 Introduction

How to make robots behave like humans has always been a hot topic. Current research has shown that human reaches are influenced by all objects in the environment and not only the target.

Thus, this project not only considers human-arms movement in single target scenarios, but also reaches in environments with multiple potential targets. Meanwhile, we plan to implement a test environment in MATLAB and use a 7-DOF robot arm model to simulate the human arm. 

## 2 Methodology

### 2.1 Movement of robot

As a first step, we should find a method to control the movement of robot arms. The movement of an object describes the relationship between position and time. Thus, by controlling the pose of robot arms, we can control their movement. The forward kinematics and the inverse kinematics can help us to do it.

### 2.2 Velocity profile similar

For single target scenarios, we need to recreate the velocity profile of human reaches. We plan to use quintic polynomial interpolation, which makes us can keep the profile of trajectories, velocity and acceleration continuous and smooth while controlling the velocity of robot arms.

### 2.3 Trajectory  profile similar

Besides, we need to solve the problem of trajectory profiles similar in multiple potential targets scenarios. We use the Sigmoid function to simulate the human attention shift in the reaching task, and then achieve the trajectories profile similar.

## 3 Summary

* I use a 7-DOF robot arm to simulate the human arm.

* I use forward kinematics and inverse kinematics to control the movement of robot arms.

* I use Pseudo inverse method for solving the Jacobian matrix inversion.

* I use Damped Least Squares method (DLS) for Singularity avoidance.

* I use the Sigmoid function to simulate the Human Attention Shift.

* I use the quintic polynomial interpolation to implement the velocity profile similar.

* I use gradient projection method (GPM) for Joint limit avoidance.

> reference

[1] Maciejewski AA, Klein CA. Obstacle Avoidance for Kinematically Redundant Manipulators in Dynamically Varying Environments. The International Journal of Robotics Research. 1985;4(3):109-117. doi:10.1177/027836498500400308 https://journals.sagepub.com/doi/10.1177/027836498500400308

[2] Xie S, Sun L, Wang Z, Chen G. A speedup method for solving the inverse kinematics problem of robotic manipulators. International Journal of Advanced Robotic Systems. 2022;19(3). doi:10.1177/17298806221104602 https://journals.sagepub.com/doi/full/10.1177/17298806221104602#bibr26-17298806221104602

[3] H. Zghal, R. V. Dubey and J. A. Euler, "Efficient gradient projection optimization for manipulators with multiple degrees of redundancy," Proceedings., IEEE International Conference on Robotics and Automation, Cincinnati, OH, USA, 1990, pp. 1006-1011 vol.2, doi: 10.1109/ROBOT.1990.126123. https://ieeexplore.ieee.org/abstract/document/126123

[4] Kou, C.X., Dai, Y.H. A Modified Self-Scaling Memoryless Broyden–Fletcher–Goldfarb–Shanno Method for Unconstrained Optimization. J Optim Theory Appl 165, 209–224 (2015). https://doi.org/10.1007/s10957-014-0528-4

[5] https://ww2.mathworks.cn/help/robotics/ug/inverse-kinematics-algorithms.html#bve7api

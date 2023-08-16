**Author:** Junhong Liu.

**Supervisor:** Dr. Dietmar Heinke & Dr. Masoumeh Iran Mansouri

**Platform:** Matlab 2022b.

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

> reference code

[1] Pseudo_inverse_method_for_7-DOF_manipulator. https://github.com/bhtxy0525/Pseudo_inverse_method_for_7-DOF_manipulator
[2] Polynomial Interpolation. https://github.com/chauby/PolynomialInterpolation

> reference paper

[1] Marr, D. (1982). Vision: A computational investigation into the human representation and processing of visual information. The MIT Press.

[2] Makwana, M., Zhang, F., Heinke, D., & Song, J. (2022). Continuous action with a neurobiologically inspired computational approach reveals the dynamics of selection history. https://doi.org/10.31234/osf.io/8xgbm

[3] Strauss, S., J.W. Woodgate, P., A. Sami, S., & Heinke, D. (2015). Choice reaching with a LEGO arm robot (CoRLEGO): The motor system guides visual attention to movement-relevant information. Neural Networks,72, 3-12.

[4] Song, J. H., & Nakayama, K. (2008). Target selection in visual search as revealed by movement trajectories. Vision Research, 48, 853–861.

[5] Lamy, D., Antebi, C., Aviani, N., Carmel, T. (2008). Priming of Pop-out provides reliable measures of target activation and distractor inhibition in selective attention. Vision Research 48, 30–41.

[6] Johnson, A., & Proctor, R. W. (2004). Attention: Theory and practice. Sage.

[7] LaBerge, D., & Brown, V. (1989). Theory of attentional operations in shape identification. Psychological Review, 96(1), 101–124.

[8] Siciliano, B., Sciavicco, L., Villani, L., Oriolo, G. (2009). Robotics Modelling, Planning and Control. Springer London.

[9] The Robotics Toolbox. https://petercorke.com/toolboxes/robotics-toolbox/

[10] Shadmehr, R., & Wise, S. P. (2004). The computational neurobiology of reaching and pointing: a foundation for motor learning. MIT press.


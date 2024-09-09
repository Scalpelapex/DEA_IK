# A Robust Surface Mesh Reconstruction Method for Real-Time 3D Visualization in Minimally Invasive Surgery: Addressing Noise, Topology, and Deformation Accuracy

Paper Link: https://doi.org/10.1016/j.measurement.2023.112925

## Abstract
Performance evaluation and inverse kinematics (IK) for continuum robots are always hard and time-consuming. In this paper, an efficient dexterity evaluation algorithm based on IK (DEAIK) for continuum robots is proposed. The IK model is established using an oval curve equation to improve computational efficiency. The relationship between length distributions and dexterity distribution is obtained by the simulation in this paper. Length distribution of the two-segment continuum robot is optimized under the guidance of the dexterity indices using the fruit fly algorithm. Theoretical analysis and numerical simulations demonstrate that the dexterity of the structure-optimized continuum robot is better than that of the traditional continuum robot. The simulation shows that the DEAIK algorithm is 3.68 times faster than the algorithm based on forward kinematics. The IK algorithm in this paper is 3 203 times faster than the IK algorithm based on the Levenberg-Marquardt algorithm in the same accuracy. This work is significant for designing a high-performance continuum robot.

# fig1 - 机器人运动gif
![](https://github.com/Scalpelapex/Images/blob/main/VC_CDPS/Overview.jpg)

## Algorithm principle
Under the assumption of equal curvature, we use the solution space of Kepler oval curve as the forward kinematics solution space of single segment continuum, and then use the curve equation to explicitly obtain the inverse kinematics equation from Cartesian space coordinates to robot configuration space coordinates. Then, we introduce the flexibility evaluation ball as the evaluation index of space flexibility, and fuse the terminal attitude angle to define the flexible operation space. The optimal design ratio of the two segments of continuum is 0.37.

# fig2 - 算法原理图

## Results

# fig3 - 结果图


## Code
Clone the repo: 
> https://github.com/zhenguonie/VFCM-CHPS

Open this project with Matlab

Code Decription:
```
(1) Main (*.mlx) file:
DEAIK_Solver.mlx -- main 
(2) Function (*.m) files:
1> b_preparation.m -- Initialization parameter b
2> End_position_solve.m -- Calculate end position of two-segment  continuum with the length of s1(first) and s2(second)
3> End_posture.m -- Calculate end posture
4> SOLVE_PE.m -- Generate sample data
5> fai_solve -- solve fai
6> theta_solve -- solve theta
7> PositionEnd -- similar to 3>
8> workspace.m -- calculate the workspace of the continuum
9> newtoniteration1 -- Newton Iteration Method

Note: Text Encoding: UTF-8
```


## Citation

If you found this code helpful, please consider citing:

```
@article{xxx,
title = {Continuum robots: Developing dexterity evaluation algorithms using efficient inverse kinematics},
journal = {Measurement},
volume = {216},
pages = {112925},
year = {2023},
issn = {0263-2241},
doi = {https://doi.org/10.1016/j.measurement.2023.112925},
author = {Fuxin Du and Gang Zhang and Yanjie Xu and Yanqiang Lei and Rui Song and Yibin Li},
}
```

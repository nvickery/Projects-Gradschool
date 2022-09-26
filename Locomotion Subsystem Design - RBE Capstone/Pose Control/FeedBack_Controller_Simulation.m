% Nathanael Vickery
% RBE594 - RBE Capstone Project - Locomotion
% 4 Wheeled Mecanum Robot Simulation 
% March 26, 2022

clear
clc 

%% Initiated Map
 load ExampleMap
 test_map = map;
 
 %% Initial Pose
 p_o = [1, 1, 0];
 init_x = p_o(1);
 init_y = p_o(2);
 init_theta = p_o(3);
 
 %% Test Trajectory
 goal_pose = [4, 6, pi/2];
 max_vbody = 1.5; % max body velocity [m/s]
 Tf = 10; % theoretical time it takes to get to goal
 
[ q_x_coef, qdot_x_coef, qddot_x_coef ] = q_traj(0, Tf, init_x, goal_pose(1), 0, 0);
[ q_y_coef, qdot_y_coef, qddot_y_coef ] = q_traj(0, Tf, init_y, goal_pose(2), 0, 0);
[ q_theta_coef, qdot_theta_coef, qddot_theta_coef ] = q_traj(0, Tf, init_theta, goal_pose(3), 0, 0);

 
 %% Run Sim
test = sim('LocomotionSubsystem_FeedbackControlTraj', 2*Tf);
 
 
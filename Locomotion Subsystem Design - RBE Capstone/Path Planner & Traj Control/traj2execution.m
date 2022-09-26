% Nathanael Vickery
% RBE594 - RBE Capstone Project - Locomotion
% 4 Wheeled Mecanum Robot Simulation - Trajectory Generator Test
% March 31, 2022

clear
clc 

%% Initiated Map
 load ExampleMap
 test_map = map;
 
 robot_rad = 0.75; % buffer radius to avoid collision
 show(test_map)
 % Collision Checking avoided in Path Planner by inflating the Occupied
 % Area
 inflate(test_map, robot_rad)
 
 %% Probabilistoc Road-Map
 path_planner = mobileRobotPRM(test_map, 525); %350);
 path_planner.ConnectionDistance = 1.5;
 update(path_planner)
% show(path_planner);
 
 %% Generate Path
 init_pose = [2, 2, 0];
 start = init_pose(1:2);
 goal_pose = [11, 10, 0];
 goal = goal_pose(1:2);
 path = findpath(path_planner, start, goal); %initial path
 show(path_planner);

 % ENSURE SUFFICIENT LENGTH FOR TIMESERIES CONTINUITY
 L = 1:length(path);
 xi = linspace(1, length(path), 500);
path = interp1(L, path, xi);
 
 dist = 0;
 for i = 1:length(path)
    if path(i,:) == path(end,:)
        dist = dist + 0;
    else
        dist = dist + sqrt((path(i+1,1) - path(i,1))^2 + (path(i+1,2) - path(i,2))^2);
    end
 end
 dist;
 
 %% Trajectory Generator from Path
 max_vel = 1.5;
 t_final = 15; % [s] dist/max_vel
 r = dist/t_final;
 time_of_arrival = transpose(zeros(1,length(path)));
 for i = 2:(length(path) - 1)
     time_of_arrival(i) = path(i)/r; 
 end
 time_of_arrival(end) = t_final;
 time_of_arrival;
 %time_of_arrival = transpose(linspace(0,t_final,length(path)))
 durat_vector = duration(0, 0, time_of_arrival);
 for i = 1:length(path)
     if path(i,:) == path(end,:)
        path(i,3) = goal_pose(3);
     elseif path(i,:) == path(1,:)
        path(i,3) = init_pose(3);
     else
        path(i,3) = goal_pose(3); %atan((path(i+1,2) - path(i,2))/(path(i+1,1) - path(i,1)));
    end
 end
 %path_dot = zeros(size(path));% 
 path_dot = [diff(path(:,1))/2, diff(path(:,2))/2, diff(path(:,3))];
 path_dot = [path_dot; 0, 0, 0];
 path = [path, path_dot];
     
 traj_series = timeseries(path, time_of_arrival);
 
 % Reset Map
 load ExampleMap
 test_map = map;
 
 %% Run Simulator to execute Desired Trajectory
 test = sim('traj2execution_example', 2*t_final);
 
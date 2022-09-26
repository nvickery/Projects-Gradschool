% Nathanael Vickery
% RBE594 - RBE Capstone Project - Locomotion
% 4 Wheeled Mecanum Robot Chassis Kinematics 
% July 26, 2021

clear
clc 

%% Robot Configuration

% This file computes the Forward and Inverse Kinematics for a 4-Wheeled
% Mecanum Robot Chassis w/ specific parameters. All kinematics are
% calculated w/in the Robot's Body Frame (Not world Frame). 

% Wheel Velocities are in the vector form as follows:
% [wheel1 (Frontleft), wheel2 (Backleft), wheel3 (Backright), wheel4 (Frontright)]

% The Robot Chassis has it's Front direction associated with it's X-axis
% and it's Left direction associated with it's Y-axis.

% The Wheels are numbered 1 through 4 starting in the +X/+Y Quadrant and
% continuing CCW.

% Refer to Locomotion_GeneralNotes.ppx for Equations of Derivation

%% Robot Parameters
syms phidot_1 phidot_2 phidot_3 phidot_4 xdot ydot thetadot

phidot = [phidot_1; phidot_2; phidot_3; phidot_4]; % wheel velocities
xidot = [xdot; ydot; thetadot]; % body velocities

r = 0.1; % wheel radius [m] 
t = 0.1; % wheel width [m]

toolbox_length = 0.735; % [m] 1" buffer of ledge distance (True length = 0.762 [m])
toolbox_width = 0.559; % [m] 1" buffer of ledge distance (True length = 0.584 [m])

wheel_base = toolbox_length - 2*r; % [m]
wheel_track = toolbox_width - t; % [m]

l = sqrt((wheel_track)^2 + (wheel_base)^2)/2; % dist from center of chassis to wheel i [m]
p = wheel_track/wheel_base; %

%% Wheel Parameters
% angle of wheel plane relative to robot's chassis [rad]
alpha = [atan(p), pi - atan(p), pi + atan(p), 2*pi - atan(p)] %
% angle of wheel i's position w.r.t robot chassis (X_r)  [rad]
beta = [pi/2 - alpha(1), 3*pi/2 + alpha(1), 3*pi/2 - alpha(1), 5*pi/2 + alpha(1)] %
% angle between wheel i's wheel plane and axis of circumferential rollers [rad]
gamma = [3*pi/4, pi/4, 3*pi/4, pi/4] %

% Rolling Constraint Matrix
J1 = [-sin(alpha(1)+beta(1)+gamma(1)), cos(alpha(1)+beta(1)+gamma(1)), l*cos(beta(1)+gamma(1));
      -sin(alpha(2)+beta(2)+gamma(2)), cos(alpha(2)+beta(2)+gamma(2)), l*cos(beta(2)+gamma(2));
      -sin(alpha(3)+beta(3)+gamma(3)), cos(alpha(3)+beta(3)+gamma(3)), l*cos(beta(3)+gamma(3));
      -sin(alpha(4)+beta(4)+gamma(4)), cos(alpha(4)+beta(4)+gamma(4)), l*cos(beta(4)+gamma(4))]; %
J1_inv = pinv(J1); %

% Wheel Contact Matrix
J2 = [r*cos(gamma(1)), 0, 0, 0;
      0, r*cos(gamma(2)), 0, 0;
      0, 0, r*cos(gamma(3)), 0;
      0, 0, 0, r*cos(gamma(4))]; %
J2_inv = pinv(J2); %


%% Forward Kinematics (w.r.t robot chassis {X_r, Y_r}) Example
Xidot = simplify(J1_inv*(-J2)*phidot)

Phidot_test_input = [-5, 5, -5, 5] %% INPUT VECTOR - WHEEL VELOCITIES

Xidot_test_output = subs(Xidot, {phidot_1, phidot_2, phidot_3, phidot_4}, {Phidot_test_input(1), Phidot_test_input(2), Phidot_test_input(3), Phidot_test_input(4)});
Xidot_test_output = Xidot_test_output

%% Inverse Kinematics (w.r.t robot chassis {X_r, Y_r}) Example
Phidot = simplify(J2_inv*(-J1)*xidot) 

Phidot_test_IK_output = subs(Phidot, {xdot, ydot, thetadot}, {Xidot_test_output(1), Xidot_test_output(2), Xidot_test_output(3)});
Phidot_test_IK_output = round(Phidot_test_IK_output)

%% Mecanum Wheeled Model
FK = J1_inv*(-J2) % Forward Kinematics (w.r.t robot chassis {X_r, Y_r})
IK  = J2_inv*(-J1) % Inverse Kinematics (w.r.t robot chassis {X_r, Y_r})

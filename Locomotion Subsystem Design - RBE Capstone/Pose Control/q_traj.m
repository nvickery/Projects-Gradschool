% Trajectory Planning - REVISED 3/26/2022

% Inputs: pd (desired position of EE)  [x; y; z]
%              qo (inital joints positions (deg)) [q1o; q2o; q3o]
% Output: q_deg ( joint positions for pd) (deg)

function [ q_coef, qdot_coef, qddot_coef ] = q_traj(ti, tf, qi_deg, qf_deg, qidot_deg, qfdot_deg)

% Polynomial Trajectory for q from ti to tf
A = [1 ti ti^2 ti^3; 0 1 2*ti 3*ti^2; 1 tf tf^2 tf^3; 0 1 2*tf 3*tf^2; ];
C = [qi_deg; qidot_deg; qf_deg; qfdot_deg];  % Zero inital velocity and final velocity
q_coef = (flip(A\C)).';
qdot_coef = [3*q_coef(1) 2*q_coef(2) q_coef(3)];
qddot_coef = [6*q_coef(1) 2*q_coef(2)];

end

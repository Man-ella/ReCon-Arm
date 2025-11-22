clear; clc; close all;

%% Robot parameters
a3 = 1;      % last link length
d1 = 0.5;      % base height
theta24_des = -pi/4;  % fixed tool orientation (this is the simplification I made)

robot = ReConArmClass(a3, d1, theta24_des);

% Initial and final Cartesian positions
pi_des = [-2; 0; 1];
pf_des = [4; 2; 3];

%% IK for initial and final pose
% IK for initial pose
sol_i = robot.ik_positionOnly(pi_des);
if isempty(sol_i)
    error('Initial position not reachable');
end
q_init = sol_i(1).q;  % pick first solution

% IK for final pose
sol_f = robot.ik_positionOnly(pf_des);
if isempty(sol_f)
    error('Final position not reachable');
end
q_final = sol_f(1).q;  % pick first solution

disp('q_init:');  disp(q_init.');
disp('q_final:'); disp(q_final.');

%% Generating joint-space cubic trajectory
t_total = 3;       % seconds
N = 50;            % number of frames
Q_traj = robot.cubicTrajectory(q_init, q_final, t_total, N);

%% Animate
figure('Name','ReCon Arm Motion');
robot.animateMotion(Q_traj, t_total);

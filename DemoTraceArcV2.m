%% demo_ReCon_arc_with_orientation.m
% ReConArm follows an arc while its tool orientation (theta2+theta4)
% varies smoothly along the path.

clear; close all; clc;

%% ---------- Robot params ----------
a3 = 1;          % last link length
d1 = 0.5;          % base height
theta24_init = -pi/2;  % initial default pitch (for initial posture)

robot = ReConArmClass(a3, d1, theta24_init);
%robot.d3_min = 0.05;   % 5 cm
%robot.d3_max = 0.30;   % 30 cm

% Initial joint state (rough)
q_init_guess = [0; 0; 0; 0.2];
robot.setJoint(q_init_guess);

%% ---------- Arc specification ----------
x_arc = 0.8;                 % fixed x coordinate
center_yz = [0.5; 1.5];      % [y_center; z_center]
radius = 0.4;

theta_start = -pi/4;
theta_end   =  pi/2;
n_wp = 30;

angles = linspace(theta_start, theta_end, n_wp);

wp = zeros(3, n_wp);
for i = 1:n_wp
    ang = angles(i);
    wp(1,i) = x_arc;                           % fixed x
    wp(2,i) = center_yz(1) + radius*cos(ang);  % y
    wp(3,i) = center_yz(2) + radius*sin(ang);  % z
end

%% ---------- Desired orientation along arc: theta24_des(i) ----------
base_pitch = -pi/2;        % base "torch down" pitch
k_orient   = 0.3;          % orientation variation factor

theta24_des_vec = zeros(1, n_wp);
ang_mid = mean([theta_start, theta_end]);

for i = 1:n_wp
    % vary around base pitch depending on deviation from arc mid-angle
    theta24_des_vec(i) = base_pitch + k_orient*(angles(i) - ang_mid);
end

%% ---------- IK per waypoint, orientation-aware ----------
q_waypoints = nan(4, n_wp);
prev_q = robot.getJoint();

for i = 1:n_wp
    p_des = wp(:,i);
    th23_des = theta24_des_vec(i);

    sols = robot.ik_pose(p_des, th23_des);
    if isempty(sols)
        error('Waypoint %d unreachable with desired orientation.', i);
    end

    % pick nearest solution to previous q for smoothness
    best_idx = 1;
    best_dist = inf;
    for k = 1:numel(sols)
        qk = sols(k).q(:);
        d = norm(qk - prev_q);
        if d < best_dist
            best_dist = d;
            best_idx = k;
        end
    end

    q_choice = sols(best_idx).q(:);
    q_waypoints(:,i) = q_choice;
    prev_q = q_choice;
end

%% ---------- Time parameterization ----------
t_total = 6.0;                     % time to complete arc
t_wp = linspace(0, t_total, n_wp); % waypoint times
T = linspace(0, t_total, 5*n_wp);  % fine time samples

% joint-space interpolation
Q_traj = zeros(4, numel(T));
for j = 1:4
    Q_traj(j,:) = interp1(t_wp, q_waypoints(j,:), T, 'pchip');
end

%% ---------- Visualization and animation ----------
figure('Name','ReCon Arc with Orientation','Color','w','Units','normalized','Position',[0.1 0.1 0.6 0.6]);
ax = axes;
hold(ax,'on');

% draw desired arc and waypoints
plot3(wp(1,:), wp(2,:), wp(3,:), 'k--','LineWidth',1.2);
plot3(wp(1,:), wp(2,:), wp(3,:), 'bo','MarkerFaceColor','b');

xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Desired Arc with Orientation Variation');
grid on; axis equal;
view(135,20);

% storage for end-effector trajectory
gif_filename = 'arc_animation.gif';
delay_time = 0.05;
P_traj = zeros(3, numel(T));

for idx = 1:numel(T)
    qk = Q_traj(:,idx);
    robot.setJoint(qk);
    [O,P1,P2,P4] = robot.jointPositions(qk);
    P_traj(:,idx) = P4;

    cla(ax);
    hold(ax,'on');

    % draw arc and waypoints again
    plot3(wp(1,:), wp(2,:), wp(3,:), 'k--','LineWidth',1.0);
    plot3(wp(1,:), wp(2,:), wp(3,:), 'bo','MarkerFaceColor','b');

    % robot links
    Xr = [O(1) P1(1) P2(1) P4(1)];
    Yr = [O(2) P1(2) P2(2) P4(2)];
    Zr = [O(3) P1(3) P2(3) P4(3)];
    plot3(Xr, Yr, Zr, '-', 'LineWidth', 3, 'Color', [0 0.4470 0.7410]);
    plot3(O(1),O(2),O(3),'ko','MarkerFaceColor','k','MarkerSize',7);
    plot3(P1(1),P1(2),P1(3),'ko','MarkerFaceColor','w','MarkerSize',7);
    plot3(P2(1),P2(2),P2(3),'ko','MarkerFaceColor','w','MarkerSize',7);

    % end-effector marker
    plot3(P4(1),P4(2),P4(3),'r*','MarkerSize',12,'LineWidth',1.6);

    % trace so far
    plot3(P_traj(1,1:idx), P_traj(2,1:idx), P_traj(3,1:idx), ...
          '-', 'Color', [0.85 0.33 0.1], 'LineWidth', 2);

    % axes
    view(45,25);
    axis equal;
    xlim([x_arc-1.0, x_arc+1.0]);
    ylim([center_yz(1)-radius-0.5, center_yz(1)+radius+0.5]);
    zlim([0, center_yz(2)+radius+0.5]);   % lower bound exactly 0
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');

    % show current theta24_des in title for debugging
    th23_now = interp1(t_wp, theta24_des_vec, T(idx), 'linear', 'extrap');
    title(sprintf('Arc Welding Motion: t = %.2f s, \\theta_{24} = %.2f rad', ...
                  T(idx), th23_now));

    drawnow limitrate;

    % Capture the current figure as an image
    frame = getframe(gcf);                 % capture figure
    im = frame2im(frame);                  % convert to image
    [imind, cm] = rgb2ind(im, 256);        % convert to indexed image
    
    % Write to the GIF file
    if idx == 1
        imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', delay_time);
    else
        imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', delay_time);
    end
   
    if idx < numel(T)
        pause(T(idx+1) - T(idx));
    end
end

disp('Arc with varying orientation complete.');

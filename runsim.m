close all;
clear;

addpath('utils');

%% pre-calculated trajectories
% trajhandle = @traj_line;
trajhandle = @traj_helix;

%% Trajectory generation with waypoints
%% You need to implement this
trajhandle = @traj_generator;
waypoints = [0    0   0;
             1    1   1;
             2    0   2;
             3    -1  1;
             4    0   0;
             8    0   2;
             15    -5  -5]';
trajhandle([],[],waypoints);


%% controller
controlhandle = @controller;


% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
[t, state] = simulation_3d(trajhandle, controlhandle);

%%
figure(1);
hold on;
scatter3(waypoints(1,:),waypoints(2,:),waypoints(3,:),...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor',[1 0 0])

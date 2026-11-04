% Given a trajectory, calculate its cost
function [Stheta, Qtheta] = stompTrajCost(model, theta, R, voxel_world, cfg)
% Compute the local trajectory cost at each discretization theta point, as
% well as the overall trajectory cost (the Qtheta)
%
% Interface change vs. original: (robot_struct, theta, R, voxel_world)
%   -> (model, theta, R, voxel_world, cfg), so FK mode, weights, safety margin and the
%   Task-5 constraint are all controlled from cfg.

[~, nDiscretize] = size(theta);
% Obstacle cost
qo_cost = zeros(1, nDiscretize);
% Constraint costs
qc_cost = zeros(1, nDiscretize);

% Get the coordinates of joints (and EE) in World frame
[X, T] = updateJointsWorldPosition(model, theta(:, 1));

% Construct the spheres around the robot manipulator for collision avoidance
[sphere_centers, radi] = stompRobotSphere(X, cfg.sphereRadius);

% Initial velocity at the sphere centers around the manipulator is 0
vel = zeros(size(sphere_centers, 1), 1);
qo_cost(1) = stompObstacleCost(sphere_centers, radi, voxel_world, vel, cfg.safetyMargin);
if cfg.useEEConstraint
    qc_cost(1) = stompConstraintCost(T{end}, cfg);
end

for i = 2 : nDiscretize
    sphere_centers_prev = sphere_centers;
    % Calculate the kinematics of the manipulator, given the
    % configuration theta values at different time (i=2:nDiscretize)
    [X, T] = updateJointsWorldPosition(model, theta(:, i));
    [sphere_centers, radi] = stompRobotSphere(X, cfg.sphereRadius);
    % Approximate the speed (xb_dot) using the finite difference of the current and
    % the previous position
    vel = vecnorm(sphere_centers_prev - sphere_centers, 2, 2);
    qo_cost(i) = stompObstacleCost(sphere_centers, radi, voxel_world, vel, cfg.safetyMargin);

    % End-effector constraint (Task 5). Original code had `qc_cost = 0;` here, which
    % overwrote the whole vector with a scalar -- now indexed per waypoint.
    if cfg.useEEConstraint
        qc_cost(i) = stompConstraintCost(T{end}, cfg);
    end
end

%% Local trajectory cost: relative weights live in cfg (w_obs, w_con)
Stheta = cfg.w_obs * qo_cost + cfg.w_con * qc_cost;

% sum over time and add the smoothness cost
thIn = theta(:, 2:end-1);
%% TODO[T1-6b] (Owner: Joshua Lim) Same cross-joint bug as in runSTOMP (see there).
Qtheta = sum(Stheta) + 1/2 * trace(thIn * R * thIn');
end

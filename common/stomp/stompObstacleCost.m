function cost = stompObstacleCost(sphere_centers, radius, voxel_world, vel, safety_margin)
% STOMPOBSTACLECOST  Obstacle cost of one waypoint, Eq. (13) of the STOMP paper:
%   q_o = sum_b max(eps + r_b - d(x_b), 0) * ||xdot_b||
% INPUT  sphere_centers nSph x 3, radius nSph x 1, vel nSph x 1 (= ||xdot_b||),
%        safety_margin = eps [m] (now passed from cfg.safetyMargin; was hard-coded 0.05
%        while the main script defined an unused safetyDistance = 0.01)

if nargin < 5
    safety_margin = 0.05;
end
cost = 0;

% signed distance function of the world
voxel_world_sEDT = voxel_world.sEDT;
world_size = voxel_world.world_size;

% calculate which voxels the sphere centers are in. idx is the grid xyz subscripts
% in the voxel world.
env_corner = voxel_world.Env_size(1, :);  % [xmin, ymin, zmin] of the metric world
idx = ceil((sphere_centers - env_corner) ./ voxel_world.voxel_size);  % implicit expansion
idx = min(max(idx,1), world_size);
lin = sub2ind(world_size, idx(:,1), idx(:,2), idx(:,3));
d = voxel_world_sEDT(lin);

% Bug fix: in the original code a sphere outside the voxel world made sub2ind error, the
% try/catch swallowed it and returned cost = 0 -- a sample that flew OUT of the workspace
% looked like the BEST sample. idx is now clamped to the grid (above).

%% Eq. (13)
cost_array = max(safety_margin + radius - d, 0) .* vel;
cost = sum(cost_array);

end


%% Construct the spheres around the robot manipulator
% Spheres are placed along the straight segment between consecutive frames:
% segment k goes from frame k-1 to frame k (frame 0 = base origin).
% INPUT:
%       X:   frames' (x,y,z,1) positions in the base frame (nFK by 4, last row = EE)
%       rad: sphere radius -- scalar, or 1 x nFK vector (one radius per segment)
%       basePos: (optional) 1x3 position of frame 0, default [0 0 0]
% OUTPUT:
%       sphere centers (nSph x 3) and radii (nSph x 1)
function [centers, radi] = stompRobotSphere(X, rad, basePos)
if nargin < 2 || isempty(rad), rad = 0.05; end
if nargin < 3 || isempty(basePos), basePos = [0 0 0]; end

nFrames = size(X, 1);
if isscalar(rad), rad = rad * ones(1, nFrames); end
assert(numel(rad) == nFrames, 'stompRobotSphere: need one radius per frame (%d).', nFrames);

center_cell = cell(nFrames, 1);
radi_cell   = cell(nFrames, 1);
for k = 1:nFrames
    if k == 1
        parent_joint_position = basePos;
    else
        parent_joint_position = X(k-1, 1:3);
    end
    child_joint_position = X(k, 1:3);

    % number of spheres to cover the kth segment: one sphere every rad(k) metres
    % The -1e-9 tolerance keeps the count independent of round-off: coincident frames
    % (Panda link1/link2, link5/link6) give a length of exactly 0 with getTransform but
    % ~1e-17 with PoE, which used to flip ceil() from 0 to 1 and change the number of
    % spheres between waypoints (-> size mismatch in stompTrajCost's velocity term).
    nSpheres = ceil(norm(child_joint_position - parent_joint_position) / rad(k) - 1e-9) + 1;

    % centres linearly spaced along the segment (vectorised version of the original arrayfun)
    s = linspace(0, 1, nSpheres)';
    center_cell{k} = parent_joint_position + s .* (child_joint_position - parent_joint_position);
    radi_cell{k}   = rad(k) * ones(nSpheres, 1);
end

% Note: for robots with large link offsets (UR elbow offsets,
% KR210 shoulder), a straight segment between joint ORIGINS may cut through air and
% miss part of the real link. Check with a plot of the spheres over show(robot) and,
% if needed, add intermediate frames (e.g. fixed bodies) to fkBodyNames.

centers = cell2mat(center_cell);
radi    = cell2mat(radi_cell);
end


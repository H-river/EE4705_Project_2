function [inCollision, worldCollisionPairIdx] = checkTrajCollision(robot, theta, world)
% CHECKTRAJCOLLISION  Ground-truth collision check of every waypoint with MATLAB's
% mesh-based checkCollision (independent of the sphere / voxel approximation used
% inside STOMP). Moved out of helperSTOMP unchanged.
nDiscretize = size(theta, 2);
inCollision = false(nDiscretize, 1);
worldCollisionPairIdx = cell(nDiscretize, 1);
for i = 1:nDiscretize
    [inCollision(i), sepDist] = checkCollision(robot, theta(:, i), world, ...
        "IgnoreSelfCollision", "on", "Exhaustive", "on");
    [bodyIdx, worldCollisionObjIdx] = find(isnan(sepDist));   % collision pairs
    worldCollisionPairIdx{i} = [bodyIdx, worldCollisionObjIdx];
end
end

function segCollides = checkTrajCollisionDense(robot, theta, world, nSub)
% CHECKTRAJCOLLISIONDENSE  Ground-truth collision check BETWEEN waypoints.
% checkTrajCollision only looks at the nDiscretize waypoints, but the robot moves
% continuously from one to the next. Each segment (waypoint i -> i+1) is interpolated
% linearly in joint space -- the same interpolation the animation uses -- with nSub - 1
% interior sub-steps, and each sub-step is checked with the mesh-based checkCollision.
%
% OUTPUT segCollides : (nDiscretize-1) x 1 logical, true if segment i collides at an
%                      interior sub-step (the waypoints themselves: checkTrajCollision)
if nargin < 4 || isempty(nSub), nSub = 10; end
N = size(theta, 2);
segCollides = false(N - 1, 1);
a = (1:nSub - 1) / nSub;
for i = 1:N - 1
    for s = 1:numel(a)
        q = theta(:, i) + a(s) * (theta(:, i + 1) - theta(:, i));
        if checkCollision(robot, q, world, "IgnoreSelfCollision", "on")
            segCollides(i) = true;
            break
        end
    end
end
end

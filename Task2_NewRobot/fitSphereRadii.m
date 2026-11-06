function [rad, info] = fitSphereRadii(model, pct, nConfigs, seed)
% FITSPHERERADII  Per-segment collision-sphere radii fitted to the robot's collision
% meshes (cfg.sphereRadius as a 1 x nFK vector, see stompRobotSphere).
%
% STOMP models the arm as spheres along the straight segments between consecutive FK
% frames (segment k: frame k-1 -> frame k, frame 0 = base origin). For nConfigs random
% configurations inside the joint limits, every vertex of every collision mesh (base,
% links, hand, fingers) is assigned to its NEAREST segment. The radius of segment k is
% the pct-th percentile of the vertex-to-segment distances assigned to it, i.e. the
% spheres contain pct % of the mesh; the rest is left to cfg.safetyMargin and to the
% mesh-based ground-truth check. Zero-length segments (coincident frames, Panda
% link1/link2 and link5/link6) get no vertices and copy the previous radius.
%
% The collision meshes are read from the rigidBodyTree itself (the same meshes
% checkCollision uses); MATLAB keeps them in an internal property, hence struct().
%
% OUTPUT rad  : 1 x nFK radii [m]
%        info : table per segment: frame, vertices, p50 / pct / max distance [m]

if nargin < 2 || isempty(pct), pct = 99; end
if nargin < 3 || isempty(nConfigs), nConfigs = 200; end
if nargin < 4 || isempty(seed), seed = 0; end

robot = model.robot;
bodies = [{robot.BaseName}, robot.BodyNames];
[V, owner] = deal({}, {});
w = warning('off', 'all');                     % struct() on objects warns
for i = 1:numel(bodies)
    if i == 1, b = robot.Base; else, b = getBody(robot, bodies{i}); end
    geoms = struct(struct(struct(b).BodyInternal).CollisionsInternal).CollisionGeometries;
    for j = 1:numel(geoms)
        g = struct(geoms{j});
        if ~isfield(g, 'CollisionPrimitive') || ~isa(g.CollisionPrimitive, 'collisionMesh'), continue; end
        P = g.CollisionPrimitive.Vertices .* g.MeshScale;              % mesh frame
        P = (g.LocalPose * [P'; ones(1, size(P, 1))])';                % body frame
        V{end + 1} = P(:, 1:3);  owner{end + 1} = bodies{i};           %#ok<AGROW>
    end
end
warning(w);
assert(~isempty(V), 'fitSphereRadii: no collision meshes found on %s.', model.name);

nFK = model.nFK;
allD = cell(1, nFK);  nVert = zeros(1, nFK);
lo = model.jointLimits(:, 1);  hi = model.jointLimits(:, 2);
lo(isinf(lo)) = -pi;  hi(isinf(hi)) = pi;
rng(seed);
for c = 1:nConfigs
    q = lo + (hi - lo) .* rand(model.nJoints, 1);
    X = updateJointsWorldPosition(model, q);
    A = [0 0 0; X(1:end - 1, 1:3)];  B = X(:, 1:3);                   % segment ends
    for i = 1:numel(V)
        if strcmp(owner{i}, robot.BaseName), T = eye(4);
        else, T = getTransform(robot, q, owner{i}); end
        P = (T * [V{i}'; ones(1, size(V{i}, 1))])';  P = P(:, 1:3);
        D = zeros(size(P, 1), nFK);
        for k = 1:nFK                                                  % point-to-segment distance
            ab = B(k, :) - A(k, :);  L2 = max(dot(ab, ab), eps);
            t = min(max(((P - A(k, :)) * ab') / L2, 0), 1);
            D(:, k) = vecnorm(P - (A(k, :) + t .* ab), 2, 2);
        end
        [d, k] = min(D, [], 2);
        for s = unique(k)'
            allD{s} = [allD{s}; d(k == s)];
            if c == 1, nVert(s) = nVert(s) + nnz(k == s); end
        end
    end
end

pick = @(x, p) prctileOrZero(x, p);
rad  = cellfun(@(x) pick(x, pct), allD);
for k = 2:nFK
    if rad(k) == 0, rad(k) = rad(k - 1); end                           % zero-length segment
end
info = table(model.fkBodyNames(:), nVert(:), cellfun(@(x) pick(x, 50), allD)', rad(:), ...
    cellfun(@(x) pick(x, 100), allD)', ...
    'VariableNames', {'segmentEndFrame', 'meshVertices', 'p50_m', sprintf('p%g_radius_m', pct), 'max_m'});
end


function v = prctileOrZero(x, p)
if isempty(x), v = 0; else, v = prctile(x, p); end
end

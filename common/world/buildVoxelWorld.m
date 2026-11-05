function [voxel_world, world] = buildVoxelWorld(scene)
% BUILDVOXELWORLD  Voxelise ALL obstacles of a scene and build the signed EDT.
% Replaces the script helperCreateObstaclesKINOVA (which hard-coded one cube).
%
% INPUT  scene.envSize    [xmin ymin zmin; lx ly lz]   (m)
%        scene.voxelSize  [dx dy dz]                   (m)
%        scene.obstacles  struct array, fields:
%            .type   'box' | 'sphere' | 'cylinder'
%            .center [x y z]                           (m, base frame)
%            .dims   box: [lx ly lz]; sphere: r; cylinder: [r h] (axis along world z)
%            .rpy    (optional, box only) [roll pitch yaw] in rad, rotation of the box
%                    about its centre, R = Rz(yaw) Ry(pitch) Rx(roll). Empty / missing
%                    = axis-aligned. (A struct array needs the field on every element,
%                    so set rpy = [] on the ones that are not rotated.)
% OUTPUT voxel_world  struct used by stompObstacleCost (same fields as original)
%        world        1 x nObs cell of collision objects (for show / checkCollision)
%
% Voxel indexing convention (same as original): binary_world(ix, iy, iz),
% voxel (ix,iy,iz) has centre  envCorner + (i - 0.5) * voxelSize.
%
% Axis-aligned boxes keep the original "every voxel the box touches" rule. Spheres,
% cylinders and rotated boxes mark the voxels whose CENTRE lies inside the shape grown
% by half a voxel (pad), so they are conservative like the box rule (a pure centre test
% lost ~18 % of a 5 cm cylinder). Each shape is only evaluated on the voxel sub-grid of
% its own bounding box (clamped to the world), not on the full grid.

envSize   = scene.envSize;
voxelSize = scene.voxelSize;
corner    = envSize(1, :);
gridSize  = round(envSize(2, :) ./ voxelSize);
binary_world = zeros(gridSize);

pad = max(voxelSize) / 2;          % growth of curved / rotated shapes (see header)

obstacles = scene.obstacles;
world = cell(1, numel(obstacles));

for i = 1:numel(obstacles)
    ob = obstacles(i);
    c  = ob.center(:)';
    switch lower(ob.type)
        case 'box'
            dims = ob.dims(:)';
            R = eye(3);
            if isfield(ob, 'rpy') && ~isempty(ob.rpy)
                %% TODO[T4-1d] rotated box
                R = eul2rotm(fliplr(ob.rpy(:)'), 'ZYX');
            end
            obj = collisionBox(dims(1), dims(2), dims(3));
            obj.Pose = [R, c'; 0 0 0 1];

            if isequal(R, eye(3))
                % ---- original cube rule, generalised; TODO[T4-1a] clamp ----
                [ix, iy, iz, ok] = idxRange(c - dims / 2, c + dims / 2, corner, voxelSize, gridSize);
                if ok, binary_world(ix, iy, iz) = 1; end
            else
                half = abs(R) * dims' / 2 + pad;                 % half extents of the AABB
                [ix, iy, iz, ok] = idxRange(c - half', c + half', corner, voxelSize, gridSize);
                if ok
                    [X, Y, Z] = voxelCentres(ix, iy, iz, corner, voxelSize);
                    P = [X(:) - c(1), Y(:) - c(2), Z(:) - c(3)] * R;   % = (R' * p)' in box frame
                    in = all(abs(P) <= dims / 2 + pad, 2);
                    binary_world(ix, iy, iz) = max(binary_world(ix, iy, iz), reshape(in, size(X)));
                end
            end

        case 'sphere'
            r = ob.dims(1);
            obj = collisionSphere(r);
            obj.Pose = trvec2tform(c);
            %% TODO[T4-1b] sphere: voxel centre within r (+pad) of the centre (ndgrid, not meshgrid)
            rp = r + pad;
            [ix, iy, iz, ok] = idxRange(c - rp, c + rp, corner, voxelSize, gridSize);
            if ok
                [X, Y, Z] = voxelCentres(ix, iy, iz, corner, voxelSize);
                in = (X - c(1)).^2 + (Y - c(2)).^2 + (Z - c(3)).^2 <= rp^2;
                binary_world(ix, iy, iz) = max(binary_world(ix, iy, iz), in);
            end

        case 'cylinder'
            r = ob.dims(1);  h = ob.dims(2);
            obj = collisionCylinder(r, h);
            obj.Pose = trvec2tform(c);
            %% TODO[T4-1c] upright cylinder: radial distance in xy <= r and |z - zc| <= h/2 (+pad)
            rp = r + pad;  hp = h / 2 + pad;
            [ix, iy, iz, ok] = idxRange(c - [rp rp hp], c + [rp rp hp], corner, voxelSize, gridSize);
            if ok
                [X, Y, Z] = voxelCentres(ix, iy, iz, corner, voxelSize);
                in = (X - c(1)).^2 + (Y - c(2)).^2 <= rp^2 & abs(Z - c(3)) <= hp;
                binary_world(ix, iy, iz) = max(binary_world(ix, iy, iz), in);
            end

        otherwise
            error('buildVoxelWorld: unknown obstacle type "%s".', ob.type);
    end
    world{i} = obj;
end

%% Signed Euclidean distance transform (unchanged from original)
voxel_world.voxel_size = voxelSize;
voxel_world.voxel      = binary_world;
voxel_world.world_size = size(binary_world);
voxel_world.Env_size   = envSize;
voxel_world.sEDT       = prod(voxelSize) ^ (1/3) * sEDT_3d(binary_world);
end


function [ix, iy, iz, ok] = idxRange(lo, hi, corner, voxelSize, gridSize)
% Voxel index ranges covering the metric interval [lo, hi], clamped to the grid
% (TODO[T4-1a]: an obstacle partly outside the world used to make sub2ind error).
% ok = false if the interval lies completely outside the world.
vLo = max(ceil((lo - corner) ./ voxelSize), 1);
vHi = min(ceil((hi - corner) ./ voxelSize), gridSize);
ok  = all(vLo <= vHi);
ix = vLo(1):vHi(1);  iy = vLo(2):vHi(2);  iz = vLo(3):vHi(3);
end

function [X, Y, Z] = voxelCentres(ix, iy, iz, corner, voxelSize)
% Metric centres of the voxel sub-grid; ndgrid keeps (x, y, z) = (dim1, dim2, dim3).
[X, Y, Z] = ndgrid(corner(1) + (ix - 0.5) * voxelSize(1), ...
                   corner(2) + (iy - 0.5) * voxelSize(2), ...
                   corner(3) + (iz - 0.5) * voxelSize(3));
end

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
% OUTPUT voxel_world  struct used by stompObstacleCost (same fields as original)
%        world        1 x nObs cell of collision objects (for show / checkCollision)
%
% Voxel indexing convention (same as original): binary_world(ix, iy, iz),
% voxel (ix,iy,iz) has centre  envCorner + (i - 0.5) * voxelSize.

envSize   = scene.envSize;
voxelSize = scene.voxelSize;
gridSize  = round(envSize(2, :) ./ voxelSize);
binary_world = zeros(gridSize);

obstacles = scene.obstacles;
world = cell(1, numel(obstacles));

for i = 1:numel(obstacles)
    ob = obstacles(i);
    switch lower(ob.type)
        case 'box'
            % ---- collision object (for visualisation / checkCollision) ----
            obj = collisionBox(ob.dims(1), ob.dims(2), ob.dims(3));
            obj.Pose = trvec2tform(ob.center);
            % ---- voxelisation (generalised from the original cube code) ----
            lo = ob.center - ob.dims / 2;
            hi = ob.center + ob.dims / 2;
            vLo = ceil((lo - envSize(1, :)) ./ voxelSize);
            vHi = ceil((hi - envSize(1, :)) ./ voxelSize);
            %% TODO[T4-1a] (Owner: Jiamo) Clamp vLo / vHi to [1, gridSize]; an
            % obstacle partly outside the voxel world currently makes sub2ind error.
            [xc, yc, zc] = ndgrid(vLo(1):vHi(1), vLo(2):vHi(2), vLo(3):vHi(3));
            binary_world(sub2ind(gridSize, xc(:), yc(:), zc(:))) = 1;

        case 'sphere'
            obj = collisionSphere(ob.dims(1));
            obj.Pose = trvec2tform(ob.center);
            %% TODO[T4-1b] (Owner: Jiamo) Voxelise a sphere.
            % Hint: build voxel-centre coordinates with ndgrid (NOT meshgrid -- meshgrid
            % swaps x/y, see the warning in the original helperCreateObstaclesKINOVA),
            % mark voxels whose centre is within r of ob.center. Pre-compute the centre
            % grids once outside the loop if several obstacles need them.
            warning('buildVoxelWorld: sphere voxelisation not implemented yet (TODO[T4-1b]).');

        case 'cylinder'
            obj = collisionCylinder(ob.dims(1), ob.dims(2));
            obj.Pose = trvec2tform(ob.center);
            %% TODO[T4-1c] (Owner: Jiamo) Voxelise an upright cylinder
            % (radial distance in xy <= r and |z - zc| <= h/2).
            warning('buildVoxelWorld: cylinder voxelisation not implemented yet (TODO[T4-1c]).');

        otherwise
            error('buildVoxelWorld: unknown obstacle type "%s".', ob.type);
    end
    world{i} = obj;
end

%% TODO[T4-1d] (Owner: Jiamo, optional) Rotated boxes (obstacle .rpy field): set
% obj.Pose with eul2tform and voxelise by transforming voxel centres into the box frame.

%% Signed Euclidean distance transform (unchanged from original)
voxel_world.voxel_size = voxelSize;
voxel_world.voxel      = binary_world;
voxel_world.world_size = size(binary_world);
voxel_world.Env_size   = envSize;
voxel_world.sEDT       = prod(voxelSize) ^ (1/3) * sEDT_3d(binary_world);
end

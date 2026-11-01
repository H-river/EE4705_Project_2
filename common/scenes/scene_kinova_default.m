function scene = scene_kinova_default()
% SCENE_KINOVA_DEFAULT  The original Task-1 setting from the sample code
% (KINOVA_STOMP_Path_Planning.m + helperCreateObstaclesKINOVA.m), as a scene struct.
%
% Scene struct (shared interface, used by every task):
%   .name        short id, used in result / video file names
%   .taskInit    4x4 initial EE pose   (or give .q0 directly in joint space)
%   .taskFinal   4x4 final EE pose     (or give .qT directly in joint space)
%   .obstacles   struct array (type / center / dims), see buildVoxelWorld
%   .envSize     [xmin ymin zmin; lx ly lz] voxel world, must contain the robot's reach
%   .voxelSize   [dx dy dz]
%   .axisLimits  axis() limits for plots
%   .viewAngle   [az el] for plots

scene.name      = 'kinova_default';
scene.taskInit  = trvec2tform([0.4 0 0.2])     * axang2tform([0 1 0 pi]);
scene.taskFinal = trvec2tform([0.35 0.55 0.35]) * axang2tform([0 1 0 pi]);

scene.obstacles = struct('type', {'box'}, ...
                         'center', {[0.4 0.46 0.26]}, ...
                         'dims', {[0.16 0.16 0.16]});

scene.envSize    = [-1, -1, -1; 2, 2, 2];
scene.voxelSize  = [0.02, 0.02, 0.02];
scene.axisLimits = [-0.8 0.8 -0.6 0.7 -0.2 0.7];
scene.viewAngle  = [150 29];
end

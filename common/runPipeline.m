function result = runPipeline(model, scene, cfg)
% RUNPIPELINE  Everything one task run needs, in order:
%   IK for start/goal -> voxel world -> initial figure -> STOMP -> ground-truth
%   collision check (init and result) -> videos / animation -> save .mat
%
% Every Task script is just: cfg + model + scene -> runPipeline.
% result = runSTOMP output plus:
%   .initInCollision          true if the straight-line init collides (Task 4 requirement)
%   .inCollision              per-waypoint collision flags of the planned trajectory
%   .isTrajectoryInCollision
%   .world, .scene, .cfg, .modelName, .fkMode

if ~exist(cfg.outDir, 'dir'), mkdir(cfg.outDir); end
tag = sprintf('%s_%s_%s', erase(model.name, {'.urdf', '/', '\'}), scene.name, model.fkMode);

%% Start / goal
[q0, qT, poseFinal] = solveInitFinal(model, scene);

%% World
[voxel_world, world] = buildVoxelWorld(scene);

%% Initial visualisation
initVisualizer(model, q0, poseFinal, world, scene);

%% Plan
result = runSTOMP(model, q0, qT, voxel_world, cfg);

%% Ground-truth collision check (mesh-based, independent of the sphere model)
initFlags = checkTrajCollision(model.robot, result.thetaInit, world);
result.initInCollision = any(initFlags);
result.inCollision = checkTrajCollision(model.robot, result.theta, world);
result.isTrajectoryInCollision = any(result.inCollision);
fprintf('[%s] init collides: %d | result collides: %d | iters: %d | time: %.2f s\n', ...
    tag, result.initInCollision, result.isTrajectoryInCollision, result.nIter, result.totalTime);

%% Videos / animation
if cfg.recordTrainingVideo
    recordTrainingVideo(model, result.theta_animation, ...
        fullfile(cfg.outDir, [tag '_training.avi']), cfg.trainingVideoStep);
end
if cfg.displayAnimation || cfg.recordResultVideo
    vidFile = '';
    if cfg.recordResultVideo, vidFile = fullfile(cfg.outDir, [tag '_result.avi']); end
    animateTrajectory(model, result.theta, vidFile);
end

%% Save
result.world     = world;
result.scene     = scene;
result.cfg       = cfg;
result.modelName = model.name;
result.fkMode    = model.fkMode;
theta = result.theta; %#ok<NASGU>  (kept for compatibility with the original .mat)
save(fullfile(cfg.outDir, [tag '_result.mat']), 'result', 'theta');
end

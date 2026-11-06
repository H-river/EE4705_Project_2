function cfg = defaultConfig()
% DEFAULTCONFIG  Single place for all STOMP / cost / visualisation parameters.
% Every Task script starts from this and overrides only what it needs, e.g.
%   cfg = defaultConfig();  cfg.useEEConstraint = true;
%
% Owner of this file: Joshua Lim (framework). Others: add new fields here rather than
% hard-coding numbers inside functions, so the report can list one parameter table.

%% STOMP
cfg.nDiscretize          = 20;    % number of waypoints (incl. fixed start / goal)
cfg.nPaths               = 20;    % noisy samples per iteration
cfg.convergenceThreshold = 0.1;   % stop when |Q_new - Q_old| < threshold
cfg.maxIter              = 50;    % hard iteration cap
cfg.h                    = 10;    % sensitivity in exp(-h * normalised cost), Eq. (11)
cfg.sigmaScale           = 1.5;   % exploration scale applied to normalised R^-1
cfg.convergencePatience  = 3;     % stop after this many CONSECUTIVE |dQ| < threshold (1 = original rule)

%% STOMP extensions (see runSTOMP; false / false / 0 / 1 = original algorithm).
% Chosen from Bonus_STOMPExtensions/stompAblation (7 scenes x 5 seeds): the limits remove
% violations of up to 0.45 rad at no loss of success; reusing 5 samples cuts the median
% iterations by 20-60 %; noise decay ALONE made T4 S1/S3 fail (0/5, 2/5) -> off.
cfg.shortestAngleInit    = true;  % continuous joints: straight line along the shortest angle
cfg.enforceJointLimits   = true;  % project samples and the updated trajectory into the limits
cfg.nReuse               = 5;     % cheapest samples of the previous iteration reused per iteration
cfg.noiseDecay           = 1;     % exploration covariance x noiseDecay^(iter-1) ...
cfg.noiseMinScale        = 0.1;   % ... but never below this fraction of the initial one

%% Obstacle cost (Eq. 13)
cfg.safetyMargin = 0.05;          % epsilon in Eq. (13) [m] (was hard-coded in stompObstacleCost)
cfg.sphereRadius = 0.05;          % scalar, or 1 x nFK vector (one radius per link)
cfg.w_obs        = 1000;          % weight of obstacle cost in S(theta)

%% End-effector orientation constraint (Task 5)
cfg.useEEConstraint = false;      % turn on in Task 5
cfg.w_con           = 0;          % weight of constraint cost in S(theta)  (Task 5 picks it by a weight sweep)
cfg.eeAxis          = 2;          % which EE body axis to constrain: 1=x, 2=y, 3=z
cfg.worldAxis       = [0; 0; 1];  % target direction in world frame (upright)

%% Kinematics
cfg.fkMode = 'getTransform';      % 'getTransform' (Task 1/2) | 'PoE' (Task 3+)

%% Ground-truth check (mesh-based checkCollision, independent of the sphere model)
cfg.denseCheckSubsteps  = 10;     % also check nSub-1 joint-space sub-steps between waypoints

%% Visualisation / output
cfg.displayAnimation    = true;
cfg.recordTrainingVideo = false;
cfg.trainingVideoStep   = 5;      % record every k-th iteration
cfg.recordResultVideo   = false;
cfg.outDir              = pwd;    % each Task script overrides with its own results/ folder
cfg.verbose             = true;
end

function result = runSTOMP(model, q0, qT, voxel_world, cfg)
% RUNSTOMP  STOMP main loop (was the script helperSTOMP.m).
%
% OUTPUT result struct:
%   .theta            nJoints x nDiscretize final trajectory
%   .thetaInit        initial (straight-line) trajectory
%   .theta_animation  1 x (nIter+1) cell, {1} = init, {k+1} = after iteration k
%   .Q_time, .RAR_time  total / control cost per iteration
%   .iterTime         wall time per iteration [s]
%   .nIter, .totalTime
%
% Extensions of the original algorithm (all switched by cfg, see defaultConfig; with
% shortestAngleInit = false, enforceJointLimits = false, nReuse = 0, noiseDecay = 1 the
% loop is exactly the original STOMP):
%   * cfg.shortestAngleInit  : continuous joints interpolated along the shortest angle
%   * cfg.enforceJointLimits : noisy samples and the updated trajectory are projected into
%                              the joint limits (original STOMP ignores them, so planned
%                              Panda trajectories left the limits by up to 0.35 rad)
%   * cfg.nReuse             : the nReuse cheapest samples of the previous iteration join
%                              the current ones (their costs are known -> no extra FK)
%   * cfg.noiseDecay, cfg.noiseMinScale : exploration covariance scaled by
%                              max(noiseDecay^(iter-1), noiseMinScale)

nDiscretize = cfg.nDiscretize;
nPaths      = cfg.nPaths;
nReuse      = getOpt(cfg, 'nReuse', 0);
noiseDecay  = getOpt(cfg, 'noiseDecay', 1);
noiseMin    = getOpt(cfg, 'noiseMinScale', 0.1);
useLimits   = getOpt(cfg, 'enforceJointLimits', false) && isfield(model, 'jointLimits');
if useLimits
    lo = model.jointLimits(:, 1);  hi = model.jointLimits(:, 2);
end

%% Initial trajectory
if getOpt(cfg, 'shortestAngleInit', false) && isfield(model, 'isContinuous')
    theta = initTrajectory(q0, qT, nDiscretize, model.isContinuous);
else
    theta = initTrajectory(q0, qT, nDiscretize);
end
thetaInit = theta;

%% Smoothing / covariance matrices (pre-computed once)
[R, sigma, M] = stompSmoothingMatrices(nDiscretize, cfg.sigmaScale);

%% Planner
[~, Qtheta] = stompTrajCost(model, theta, R, voxel_world, cfg);
QthetaOld = 0;

Q_time   = [];
RAR_time = [];
iterTime = [];
theta_animation = {theta};
dtheta_smoothed = zeros(nDiscretize - 2, model.nJoints);

nSmall = 0;      % consecutive iterations with |dQ| below the threshold
reusePaths = {};  reuseS = [];   % samples carried over from the previous iteration
iter   = 0;
tTotal = tic;
while true
    iter = iter + 1;
    QthetaOld = Qtheta;
    tIter = tic;

    %% (a) Sample noisy trajectories theta + eps, eps ~ N(0, sigma) per joint
    sigmaIt = sigma * max(noiseDecay^(iter - 1), noiseMin);   % = sigma when noiseDecay = 1
    [theta_paths, em] = stompSamples(nPaths, sigmaIt, theta);
    if useLimits
        % project every sample into the joint limits; its noise becomes (clamped - theta)
        for k = 1:nPaths
            theta_paths{k} = min(max(theta_paths{k}, lo), hi);
            for m = 1:model.nJoints
                em{m}(k, :) = theta_paths{k}(m, :) - theta(m, :);
            end
        end
    end

    %% (b) Local cost S(theta_k, i) of every sample at every waypoint
    Stheta = zeros(nPaths, nDiscretize);
    for k = 1:nPaths
        Stheta(k, :) = stompTrajCost(model, theta_paths{k}, R, voxel_world, cfg);
    end

    %% (b') Sample reuse: add the cheapest samples of the previous iteration. Their costs
    % only depend on the sample itself, so they are reused as they are; their noise is
    % re-expressed relative to the current mean trajectory.
    if nReuse > 0 && ~isempty(reusePaths)
        for r = 1:numel(reusePaths)
            for m = 1:model.nJoints
                em{m}(end + 1, :) = reusePaths{r}(m, :) - theta(m, :);
            end
        end
        theta_paths = [theta_paths, reusePaths]; %#ok<AGROW>
        Stheta      = [Stheta; reuseS];          %#ok<AGROW>
    end
    if nReuse > 0
        [~, order]  = sort(sum(Stheta, 2));
        keep        = order(1:min(nReuse, numel(order)));
        reusePaths  = theta_paths(keep);
        reuseS      = Stheta(keep, :);
    end

    %% (c) Per-waypoint probabilities P(theta_k, i), Eq. (11)
    trajProb = stompUpdateProb(Stheta, cfg.h);

    %% (d) Probability-weighted noise -> smoothed update of theta
    dtheta = stompDTheta(trajProb, em);
    [theta, dtheta_smoothed] = stompUpdateTheta(theta, dtheta, M);
    if useLimits
        theta = min(max(theta, lo), hi);   % the smoothed update can also leave the limits
    end

    %% (e) Total cost Q(theta) of the updated trajectory
    [~, Qtheta] = stompTrajCost(model, theta, R, voxel_world, cfg);

    iterTime(end + 1) = toc(tIter); %#ok<AGROW>

    %% Logging
    Q_time(end + 1) = Qtheta; %#ok<AGROW>
    thIn = theta(:, 2:nDiscretize - 1);
    % Control cost = sum_j th_j' R th_j = trace (original summed cross-joint terms too)
    RAR = 1/2 * trace(thIn * R * thIn');
    RAR_time(end + 1) = RAR; %#ok<AGROW>
    theta_animation{end + 1} = theta; %#ok<AGROW>

    if cfg.verbose
        fprintf('iter %3d | Q = %10.4f | control = %8.4f | %.3f s\n', iter, Qtheta, RAR, iterTime(end));
    end

    %% Stopping criteria
    % Patience rule: stop after cfg.convergencePatience CONSECUTIVE iterations with
    % |dQ| < threshold (patience = 1 reproduces the original single-step rule).
    if abs(Qtheta - QthetaOld) < cfg.convergenceThreshold
        nSmall = nSmall + 1;
    else
        nSmall = 0;
    end
    if nSmall >= cfg.convergencePatience
        fprintf('Converged: |dQ| < %.3g for %d consecutive iterations.\n', ...
                cfg.convergenceThreshold, nSmall);
        break
    end
    if iter >= cfg.maxIter
        fprintf('Maximum iteration (%d) reached.\n', cfg.maxIter);
        break
    end
    % (original used sum(...)==0, which can be 0 when positive and negative entries cancel)
    if all(dtheta_smoothed == 0, 'all')
        disp('Estimated gradient is 0 and theta is not updated: there could be no obstacle at all.');
        break
    end
end

result.theta           = theta;
result.thetaInit       = thetaInit;
result.theta_animation = theta_animation;
result.Q_time          = Q_time;
result.RAR_time        = RAR_time;
result.iterTime        = iterTime;
result.nIter           = iter;
result.totalTime       = toc(tTotal);
result.q0 = q0;
result.qT = qT;
disp('STOMP Finished.');
end


function v = getOpt(s, name, default)
% GETOPT  s.(name) if present and non-empty, otherwise the default.
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end

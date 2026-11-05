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

nDiscretize = cfg.nDiscretize;
nPaths      = cfg.nPaths;

%% Initial trajectory
theta = initTrajectory(q0, qT, nDiscretize);
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
iter   = 0;
tTotal = tic;
while true
    iter = iter + 1;
    QthetaOld = Qtheta;
    tIter = tic;

    %% (a) Sample noisy trajectories theta + eps, eps ~ N(0, sigma) per joint
    [theta_paths, em] = stompSamples(nPaths, sigma, theta);

    %% (b) Local cost S(theta_k, i) of every sample at every waypoint
    Stheta = zeros(nPaths, nDiscretize);
    for k = 1:nPaths
        Stheta(k, :) = stompTrajCost(model, theta_paths{k}, R, voxel_world, cfg);
    end

    %% (c) Per-waypoint probabilities P(theta_k, i), Eq. (11)
    trajProb = stompUpdateProb(Stheta, cfg.h);

    %% (d) Probability-weighted noise -> smoothed update of theta
    dtheta = stompDTheta(trajProb, em);
    [theta, dtheta_smoothed] = stompUpdateTheta(theta, dtheta, M);

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

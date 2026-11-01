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

iter = 0;
tTotal = tic;
while abs(Qtheta - QthetaOld) > cfg.convergenceThreshold
    iter = iter + 1;
    QthetaOld = Qtheta;
    tIter = tic;

    %% TODO[T1-1a] (Owner: T12) Sample noisy trajectories
    % -> stompSamples(nPaths, sigma, theta) returns theta_paths (1 x nPaths cell) and em.
    theta_paths = repmat({theta}, 1, nPaths);          % placeholder
    em = repmat({zeros(nPaths, nDiscretize)}, 1, model.nJoints); % placeholder

    %% TODO[T1-1b] (Owner: T12) Local trajectory cost S(theta_k) of every sample
    % -> loop k = 1:nPaths, Stheta(k,:) = stompTrajCost(model, theta_paths{k}, R, voxel_world, cfg)
    % (Optional speed-up for the report: parfor over samples.)
    Stheta = zeros(nPaths, nDiscretize);

    %% TODO[T1-1c] (Owner: T12) Per-time-step probabilities P(theta_k, i), Eq. (11)
    % -> stompUpdateProb(Stheta, cfg.h)
    trajProb = zeros(nPaths, nDiscretize);              % placeholder

    %% TODO[T1-1d] (Owner: T12) Gradient estimate and smoothed update
    % -> stompDTheta(trajProb, em), then stompUpdateTheta(theta, dtheta, M)
    %    (stompUpdateTheta returns both the new theta and dtheta_smoothed)

    %% TODO[T1-1e] (Owner: T12) Total cost of the updated trajectory
    % -> [~, Qtheta] = stompTrajCost(model, theta, R, voxel_world, cfg);

    iterTime(end + 1) = toc(tIter); %#ok<AGROW>

    %% Logging
    Q_time(end + 1) = Qtheta; %#ok<AGROW>
    thIn = theta(:, 2:nDiscretize - 1);
    %% TODO[T1-6b] (Owner: T12) BUG in original: sum(thIn*R*thIn','all') also adds the
    % cross-joint terms th_i' R th_j (i ~= j). The control cost in the paper is
    % sum_j th_j' R th_j = trace(thIn*R*thIn'). Same bug inside stompTrajCost.
    % Fix both, and mention it in the report as a bug fix.
    RAR = 1/2 * sum(thIn * R * thIn', 'all');
    RAR_time(end + 1) = RAR; %#ok<AGROW>
    theta_animation{end + 1} = theta; %#ok<AGROW>

    if cfg.verbose
        fprintf('iter %3d | Q = %10.4f | control = %8.4f | %.3f s\n', iter, Qtheta, RAR, iterTime(end));
    end

    %% Stopping criteria
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

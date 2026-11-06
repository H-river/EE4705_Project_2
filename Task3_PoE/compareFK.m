function stats = compareFK(robotName, eeName, nTest)
% COMPAREFK  Accuracy and speed of PoE FK vs MATLAB getTransform on random configs.
% Owner: Jiamo (T34)
%
% stats (for the report table):
%   .maxPosErr   max over configs & frames of ||p_PoE - p_GT||          [m]
%   .maxRotErr   max over configs & frames of ||R_PoE' R_GT - I||_F
%   .tGT, .tPoE  mean time of ONE full FK call (all frames)              [s]
%   .tNaive      same, but PoE done the "textbook" way: one FKinSpace-style product
%                with expm per frame (O(n^2) exponentials) -- shows why fkPoE reuses
%                the running product and uses the closed-form exponential
%   .speedup     tGT / tPoE

if nargin < 3, nTest = 500; end

mGT  = buildRobotModel(robotName, eeName, 'getTransform');
mPoE = buildRobotModel(robotName, eeName, 'PoE');

%% Random configurations inside joint limits
lim = zeros(mGT.nJoints, 2);
for i = 1:mGT.nJoints
    lim(i, :) = mGT.robot.getBody(mGT.jointBodyNames{i}).Joint.PositionLimits;
end
lim(isinf(lim)) = sign(lim(isinf(lim))) * pi;    % continuous joints -> [-pi, pi]
Q = lim(:, 1) + rand(mGT.nJoints, nTest) .* (lim(:, 2) - lim(:, 1));

%% Accuracy -- every frame k = 1..nFK (the planner uses all of them)
maxPos = 0;  maxRot = 0;
for t = 1:nTest
    [~, Tg] = updateJointsWorldPosition(mGT,  Q(:, t));
    [~, Tp] = updateJointsWorldPosition(mPoE, Q(:, t));
    for k = 1:mGT.nFK
        maxPos = max(maxPos, norm(Tp{k}(1:3, 4) - Tg{k}(1:3, 4)));
        maxRot = max(maxRot, norm(Tp{k}(1:3, 1:3)' * Tg{k}(1:3, 1:3) - eye(3), 'fro'));
    end
end
stats.maxPosErr = maxPos;
stats.maxRotErr = maxRot;

%% Speed (median of a few configs, each timed with timeit)
nTime = 5;
tg = zeros(1, nTime);  tp = tg;  tn = tg;
for t = 1:nTime
    q = Q(:, t);
    tg(t) = timeit(@() updateJointsWorldPosition(mGT,  q));
    tp(t) = timeit(@() updateJointsWorldPosition(mPoE, q));
    tn(t) = timeit(@() naivePoEAllFrames(mPoE.S, mPoE.Mhome, q));
end
stats.tGT    = median(tg);
stats.tPoE   = median(tp);
stats.tNaive = median(tn);
stats.speedup = stats.tGT / stats.tPoE;
stats.robot   = robotName;
stats.nTest   = nTest;

fprintf('[compareFK] %s | maxPosErr %.2e m | maxRotErr %.2e | GT %.1f us | PoE %.1f us | x%.1f | naive PoE (expm per frame) %.1f us\n', ...
    robotName, stats.maxPosErr, stats.maxRotErr, 1e6*stats.tGT, 1e6*stats.tPoE, stats.speedup, 1e6*stats.tNaive);
end

function T = naivePoEAllFrames(S, Mhome, theta)
% Textbook PoE per frame (like calling Modern Robotics FKinSpace once per frame):
% every frame rebuilds its own product with the general matrix exponential.
n = size(S, 2);  nFK = size(Mhome, 3);
T = cell(1, nFK);
for k = 1:nFK
    E = eye(4);
    for i = 1:min(k, n)
        w = S(1:3, i);  v = S(4:6, i);
        se3 = [0, -w(3), w(2), v(1); w(3), 0, -w(1), v(2); -w(2), w(1), 0, v(3); 0 0 0 0];
        E = E * expm(se3 * theta(i));
    end
    T{k} = E * Mhome(:, :, k);
end
end

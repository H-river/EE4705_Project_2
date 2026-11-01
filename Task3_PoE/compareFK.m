function stats = compareFK(robotName, eeName, nTest)
% COMPAREFK  Accuracy and speed of PoE FK vs MATLAB getTransform on random configs.
% Owner: Jiamo (T34)
%
% stats (for the report table):
%   .maxPosErr   max over configs & frames of ||p_PoE - p_GT||          [m]
%   .maxRotErr   max over configs & frames of ||R_PoE' R_GT - I||_F
%   .tGT, .tPoE  mean time of ONE full FK call (all frames)              [s]
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

stats.maxPosErr = NaN;
stats.maxRotErr = NaN;
stats.tGT  = NaN;
stats.tPoE = NaN;
stats.speedup = NaN;

%% TODO[T3-3a] (Owner: Jiamo) Accuracy
% for each column of Q: [~, Tg] = updateJointsWorldPosition(mGT, q);
%                       [~, Tp] = updateJointsWorldPosition(mPoE, q);
% compare every frame k = 1..nFK (not only the EE -- the planner uses all frames).
% NOTE: needs TODO[T1-2] (getTransform branch) done first; until then call
%       getTransform(mGT.robot, q, mGT.fkBodyNames{k}) directly here.

%% TODO[T3-3b] (Owner: Jiamo) Speed
% stats.tGT  = timeit(@() updateJointsWorldPosition(mGT,  Q(:,1)));
% stats.tPoE = timeit(@() updateJointsWorldPosition(mPoE, Q(:,1)));
% Optionally also time: the ORIGINAL struct-config getTransform (original_code/),
% and Modern Robotics FKinSpace called per frame, to show why our fkPoE is faster.

fprintf('[compareFK] %s | maxPosErr %.2e m | maxRotErr %.2e | GT %.1f us | PoE %.1f us | x%.1f\n', ...
    robotName, stats.maxPosErr, stats.maxRotErr, 1e6*stats.tGT, 1e6*stats.tPoE, stats.speedup);
end

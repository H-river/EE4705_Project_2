function theta = initTrajectory(q0, qT, nDiscretize, isContinuous)
% INITTRAJECTORY  Initial guess: straight line in joint space from q0 to qT.
% Task 4 rule: this initial guess must NOT be collision-free (main_task4 checks it).
%
% isContinuous (optional, 1 x nJoints logical, model.isContinuous): continuous joints are
% wrapped to [-pi, pi] by solveInitFinal, so a plain linspace can go "the long way
% round" (e.g. 2.8 -> -2.8 passes through 0: 5.6 rad instead of 0.68 rad, and the arm
% sweeps almost a full turn). For these joints the line follows the SHORTEST angular
% difference wrapToPi(qT - q0) (handout tip on joint-angle discontinuity). The last
% waypoint is then q0 + wrapToPi(qT - q0): the same physical angle as qT, possibly
% outside [-pi, pi], which is fine for a joint without limits.

nJoints = numel(q0);
d = qT(:) - q0(:);
if nargin >= 4 && ~isempty(isContinuous)
    c = logical(isContinuous(:));
    d(c) = wrapToPi(d(c));
end
theta = zeros(nJoints, nDiscretize);
for k = 1:nJoints
    theta(k, :) = linspace(q0(k), q0(k) + d(k), nDiscretize);
end
end

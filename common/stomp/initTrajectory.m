function theta = initTrajectory(q0, qT, nDiscretize)
% INITTRAJECTORY  Initial guess: straight line in joint space from q0 to qT.
% Task 4 rule: this initial guess must NOT be collision-free (main_task4 checks it).

nJoints = numel(q0);
theta = zeros(nJoints, nDiscretize);
for k = 1:nJoints
    theta(k, :) = linspace(q0(k), qT(k), nDiscretize);
end

% Possible improvement (not implemented; handout tip): joints wrapped to [-pi, pi] can make the
% straight line go "the long way round" (e.g. 3.1 -> -3.1 passes through 0). For
% continuous joints, interpolate along the shortest angular difference
% (wrapToPi(qT - q0)) and mention it in the report as an improvement.
end

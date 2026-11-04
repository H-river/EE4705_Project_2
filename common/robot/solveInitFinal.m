function [q0, qT, poseFinal] = solveInitFinal(model, scene)
% SOLVEINITFINAL  Start / goal joint configurations for a scene.
% If the scene gives scene.q0 / scene.qT directly (joint space), they are used as-is;
% otherwise IK is solved from scene.taskInit / scene.taskFinal (as in the original code).
%
% poseFinal = [x; y; z; eulXYZ] of the goal, only used for plotting the red target dot.

weights = [1 1 1 1 1 1];
ik = inverseKinematics('RigidBodyTree', model.robot);
ik.SolverParameters.AllowRandomRestart = true;
rngState = rng;
rng(0);

%% Start
if isfield(scene, 'q0') && ~isempty(scene.q0)
    q0 = scene.q0(:);
else
    [q0, ikInfo] = ik(model.eeName, scene.taskInit, weights, homeConfiguration(model.robot));   % NEW: 2 outputs
    if ~strcmp(ikInfo.Status, 'success')                                                         % NEW
        warning('solveInitFinal: start IK status "%s" (pose error %.4f)', ikInfo.Status, ikInfo.PoseErrorNorm);
    end
end

%% Goal
if isfield(scene, 'qT') && ~isempty(scene.qT)
    qT = scene.qT(:);
else
    [qT, ikInfo] = ik(model.eeName, scene.taskFinal, weights, q0);                               % NEW: 2 outputs
    if ~strcmp(ikInfo.Status, 'success')                                                         % NEW
        warning('solveInitFinal: goal IK status "%s" (pose error %.4f)', ikInfo.Status, ikInfo.PoseErrorNorm);
    end
end

rng(rngState);
%% Wrap continuous / revolute joints into [-pi, pi]
%% TODO[T2-3] (Owner: Joshua Lim) If the chosen robot has a PRISMATIC joint, do not wrap it.
% Original code called wrapToPi on the whole vector, which is wrong for prismatic joints.
isAngular = ~strcmp(model.jointTypes, 'prismatic')';
q0(isAngular) = wrapToPi(q0(isAngular));
qT(isAngular) = wrapToPi(qT(isAngular));

%% Goal pose for plotting
Tgoal = getTransform(model.robot, qT, model.eeName);
poseFinal = [Tgoal(1:3, 4); rotm2eul(Tgoal(1:3, 1:3), 'XYZ')'];

%% Sanity check: IK accuracy (IK may silently fail for unreachable poses)
if isfield(scene, 'taskFinal') && ~isempty(scene.taskFinal)
    err = norm(Tgoal(1:3, 4) - scene.taskFinal(1:3, 4));
    if err > 1e-3
        warning('solveInitFinal: goal IK position error = %.4f m (unreachable pose?)', err);
    end
end
end

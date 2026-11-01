function scene = scene_newrobot_base()
% SCENE_NEWROBOT_BASE  Task-2 scene: "similar scenario to Task 1" for the chosen robot.
% Tasks 3 and 5 reuse this scene unless they define their own.

rc = robotConfig();
scene = scene_kinova_default();   % start from the Task-1 scene
scene.name = ['task2_' rc.robotName];

if ~rc.isDecided
    % Robot not chosen yet -> this is just the Kinova scene (lets Tasks 3-5 start).
    return
end

%% TODO[T2-4] (Owner: T12) Adapt the scene to the chosen robot.
% - Initial / final EE poses inside the new robot's reach (check IK error printed by
%   solveInitFinal), and the final pose must itself be collision-free (handout tip).
% - Move / resize the box so that the straight-line joint interpolation still
%   collides (otherwise the planning problem is trivial).
% - Enlarge envSize if the robot reaches beyond [-1, 1]^3 (e.g. KUKA KR210).
% - Update axisLimits / viewAngle for good figures.
% scene.taskInit  = ...;
% scene.taskFinal = ...;
% scene.obstacles = ...;
end

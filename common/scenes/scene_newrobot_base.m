function scene = scene_newrobot_base()
% SCENE_NEWROBOT_BASE  Task-2 scene: "similar scenario to Task 1" for the chosen robot.
% Tasks 3 and 5 reuse this scene unless they define their own.

rc = robotConfig();
scene = scene_kinova_default();   % start from the Task-1 scene
scene.name = ['task2_' rc.robotName];

if ~rc.isDecided
    return      % robot not chosen yet -> plain Kinova Task-1 scene
end

% Task-2 scene for the Panda (TCP = fingertips): goal chosen for IK success + clearance;
% box placed on the joint-space straight-line path (waypoint 12/20) so the initial guess
% collides. Mesh clearance: 9.2 cm at start, 13.3 cm at goal, min 5.4 cm along the plan.
 scene.taskInit  = trvec2tform([0.4, 0, 0.2]) * axang2tform([0 1 0 pi]);   % same start as Task 1
 scene.taskFinal = trvec2tform([0.3, 0.60, 0.45]) * axang2tform([0 1 0 pi]);   % goal 2
 scene.obstacles = struct('type', {'box'}, ...
                             'center', {[0.47, 0.31, 0.46]}, ...                    % wp12 centre
                             'dims',   {[0.16 0.16 0.16]});
 scene.axisLimits = [-0.8 0.8 -0.6 0.8 -0.2 1.0];   % taller than Kinova: Panda + box top at 0.54 m

end

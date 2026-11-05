function scene = scene_t4_3()
% SCENE_T4_3  Task-4 custom scenario 3 -- large re-orientation around a column, a ball and a tilted panel
% Owner: Jiamo (T34)
%
% Start in front of the robot with the tool pointing forward, goal on the left with the
% tool pointing sideways: large changes in joints 1 and 7. A column (cylinder) and a
% ball (sphere) sit on the swept path and a rotated panel (box, yaw 30 deg) closes the
% high route, so the arm has to thread between them.
% Tuning (PoE, maxIter 200, seeds 1-5): goal at z = 0.35 -> 0/5 (the hand stays hooked on
% the column, a local minimum STOMP's obstacle cost already sees); goal at z = 0.50 -> 5/5.

scene = scene_newrobot_base();       % robot-specific defaults (envSize, view, ...)
scene.name = 't4_3';
if isfield(scene, 'q0'), scene = rmfield(scene, 'q0'); end   % use taskInit / taskFinal
if isfield(scene, 'qT'), scene = rmfield(scene, 'qT'); end

down    = axang2tform([0 1 0 pi]);        % tool z pointing down
forward = axang2tform([0 1 0 pi/2]);      % tool z pointing along +x
side    = axang2tform([1 0 0 -pi/2]);     % tool z pointing along +y

scene.description = 'Large re-orientation (front -> left side) around a column, a ball and a rotated panel.';
scene.taskInit  = trvec2tform([0.50, -0.25, 0.35]) * forward;
scene.taskFinal = trvec2tform([0.10,  0.55, 0.50]) * side;
scene.obstacles = struct('type',   {'cylinder', 'sphere', 'box'}, ...
                         'center', {[0.45 0.22 0.35], [0.30 0.35 0.12], [0.15 0.35 0.75]}, ...
                         'dims',   {[0.06 0.70], 0.08, [0.40 0.04 0.20]}, ...
                         'rpy',    {[], [], [0 0 pi/6]});
end

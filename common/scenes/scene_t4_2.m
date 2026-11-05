function scene = scene_t4_2()
% SCENE_T4_2  Task-4 custom scenario 2 -- reach under a shelf (box plate + cylinder legs)
% Owner: Jiamo (T34)
%
% Start above a shelf, goal under it with the tool horizontal (like taking a part from
% a lower shelf). The straight-line interpolation drops the hand through the shelf
% plate; the planner must pull the arm out in front of the plate edge and come in below.
% Tuning (PoE, seeds 1-3): 3/3 collision-free with the default 50 iterations.

scene = scene_newrobot_base();       % robot-specific defaults (envSize, view, ...)
scene.name = 't4_2';
if isfield(scene, 'q0'), scene = rmfield(scene, 'q0'); end   % use taskInit / taskFinal
if isfield(scene, 'qT'), scene = rmfield(scene, 'qT'); end

down    = axang2tform([0 1 0 pi]);        % tool z pointing down
forward = axang2tform([0 1 0 pi/2]);      % tool z pointing along +x
side    = axang2tform([1 0 0 -pi/2]);     %#ok<NASGU> tool z pointing along +y

scene.description = 'Start above a shelf, goal under it (tool horizontal): arm must go round the plate edge.';
scene.taskInit  = trvec2tform([0.45, -0.10, 0.62]) * down;
scene.taskFinal = trvec2tform([0.55,  0.10, 0.22]) * forward;
scene.obstacles = struct('type',   {'box', 'cylinder', 'cylinder'}, ...
                         'center', {[0.55 0 0.40], [0.67 -0.32 0.20], [0.67 0.32 0.20]}, ...
                         'dims',   {[0.30 0.70 0.03], [0.02 0.40], [0.02 0.40]}, ...
                         'rpy',    {[], [], []});
scene.viewAngle = [20 15];           % side view: shows the detour round the plate edge
end

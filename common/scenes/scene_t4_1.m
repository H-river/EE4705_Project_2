function scene = scene_t4_1()
% SCENE_T4_1  Task-4 custom scenario 1 -- over a low wall (box) next to a post (cylinder)
% Owner: Jiamo (T34)
%
% Start and goal are on the two sides of a low wall. The straight-line joint
% interpolation (mostly joint 1) sweeps the hand straight through the wall; going round
% the inner end of the wall is blocked by a post, so the planner must lift the arm over.
% Tuning (PoE, maxIter 200): wall 34 cm high -> 5/8 collision-free, 28 cm -> 15/20,
% 25 cm -> 18/20 (seeds 1-20).
% Start/goal at z = 0.18 instead of 0.12 made the straight line (almost) collision-free,
% i.e. a trivial problem, so the hand stays low.

scene = scene_newrobot_base();       % robot-specific defaults (envSize, view, ...)
scene.name = 't4_1';
if isfield(scene, 'q0'), scene = rmfield(scene, 'q0'); end   % use taskInit / taskFinal
if isfield(scene, 'qT'), scene = rmfield(scene, 'qT'); end

down    = axang2tform([0 1 0 pi]);        % tool z pointing down
forward = axang2tform([0 1 0 pi/2]);      %#ok<NASGU> tool z pointing along +x
side    = axang2tform([1 0 0 -pi/2]);     %#ok<NASGU> tool z pointing along +y

scene.description = 'Low wall between start and goal, post blocks the inner end: arm must lift over the wall.';
scene.taskInit  = trvec2tform([0.45, -0.35, 0.12]) * down;
scene.taskFinal = trvec2tform([0.45,  0.35, 0.12]) * down;
scene.obstacles = struct('type',   {'box', 'cylinder'}, ...
                         'center', {[0.50 0 0.125], [0.24 0 0.17]}, ...
                         'dims',   {[0.45 0.04 0.25], [0.03 0.34]}, ...
                         'rpy',    {[], []});
scene.viewAngle = [30 20];           % look at the wall face: shows the lift over it
end

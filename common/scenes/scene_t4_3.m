function scene = scene_t4_3()
% SCENE_T4_3  Task-4 custom scenario 3 -- e.g. different start/goal configs that need a large joint-space detour.
% Owner: Jiamo (T34)

scene = scene_newrobot_base();       % robot-specific defaults (envSize, view, ...)
scene.name = 't4_3';

%% TODO[T4-2] (Owner: Jiamo) Design scenario 3.
% Requirements from the handout / grading:
%   * multiple obstacles (box / sphere / cylinder), different init & final targets;
%   * NON-TRIVIAL initialisation: the straight-line joint interpolation must collide
%     -- main_task4 prints result.initInCollision, it must be true;
%   * goal pose itself collision-free and reachable (watch the IK warning).
% Describe in one line why this scenario is hard (goes into the report table):
scene.description = 'TODO';
% scene.taskInit  = trvec2tform([...]) * axang2tform([...]);   % or scene.q0 = [...]
% scene.taskFinal = trvec2tform([...]) * axang2tform([...]);   % or scene.qT = [...]
% scene.obstacles = struct('type',   {'box', 'sphere'}, ...
%                          'center', {[x y z], [x y z]}, ...
%                          'dims',   {[lx ly lz], r});
end

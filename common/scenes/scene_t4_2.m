function scene = scene_t4_2()
% SCENE_T4_2  Task-4 custom scenario 2 -- e.g. reach over or under a shelf / table (cluttered workspace).
% Owner: Jiamo (T34)

scene = scene_newrobot_base();       % robot-specific defaults (envSize, view, ...)
scene.name = 't4_2';

%% TODO[T4-2] (Owner: Jiamo) Design scenario 2.
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

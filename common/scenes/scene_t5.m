function scene = scene_t5()
% SCENE_T5  Task-5 scene: same obstacles as Task 2 but start / goal EE orientations
% chosen so the constrained axis is already upright (handout: "adjust the initial and
% final orientation of the end-effector to align with your desired orientation").
% Owner: T5

scene = scene_newrobot_base();
scene.name = 't5_upright';

%% TODO[T5-2a] (Owner: T5) Set taskInit / taskFinal with the EE axis cfg.eeAxis
% aligned to cfg.worldAxis (= world z). E.g. build the rotation with columns
% [x_b, y_b, z_b] where y_b = [0 0 1]' and x_b, z_b horizontal, then
% scene.taskInit = [R p; 0 0 0 1]. Verify with getTransform after IK.
% scene.taskInit  = ...;
% scene.taskFinal = ...;
end

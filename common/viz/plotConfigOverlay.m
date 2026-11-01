function fig = plotConfigOverlay(model, theta, world, scene, opts)
% PLOTCONFIGOVERLAY  Static figure for report/slides: intermediate configurations of
% a trajectory overlaid in one plot (handout Task 4: "overlapping the intermediate
% configurations"). Used by Tasks 2-5 -> keep the call signature stable.
%
% INPUT  theta : nJoints x nDiscretize
%        world : collision objects (from buildVoxelWorld)
%        scene : for axisLimits / viewAngle / name
%        opts  : (optional) struct, suggested fields
%                .stride   plot every k-th waypoint (default 2)
%                .showEEPath  draw the EE path as a line (default true)
%                .title, .saveFile ('' = do not save; e.g. 'fig.png')
%
% Owner: T5

if nargin < 5, opts = struct(); end
fig = figure('Name', ['overlay_' scene.name]);

%% TODO[T5-3a] (Owner: T5) Implement.
% Hints:
%   ax = show(model.robot, theta(:,1), 'PreservePlot', true, 'Frames', 'off'); hold on
%   loop t = 1:stride:N, show(..., 'Parent', ax, 'PreservePlot', true, 'Frames','off')
%   - fade colour from start to goal: after each show(), find the newly added patch
%     objects (findobj(ax,'Type','patch')) and set FaceAlpha / FaceColor;
%   - EE path: getTransform(model.robot, theta(:,t), model.eeName) for all t, plot3;
%   - obstacles: show(world{i}, 'Parent', ax);
%   - axis(scene.axisLimits); view(scene.viewAngle); exportgraphics(fig, opts.saveFile).
% Make it look the same for every task -- the slides will reuse these figures.
end

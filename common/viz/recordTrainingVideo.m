function recordTrainingVideo(model, theta_animation, videoFile, step)
% RECORDTRAININGVIDEO  Video of how the trajectory evolves over STOMP iterations.
% Fixes vs. original helperSTOMP:
%   * the VideoWriter was opened (creating an empty .avi) even when disabled;
%   * k = 0:5:nTraining with theta_animation{k+1} indexed past the end whenever
%     nTraining was a multiple of 5.
if nargin < 4, step = 5; end

nTraining = numel(theta_animation);   % {1} = initial trajectory
v = VideoWriter(videoFile);
v.FrameRate = 15;
open(v);
htext = text(-0.2, 0.6, 0.7, 'Iteration = 0', 'HorizontalAlignment', 'left', 'FontSize', 14);

for k = 1:step:nTraining
    set(htext, 'String', sprintf('Iteration = %d', k - 1));
    theta_tmp = theta_animation{k};
    for t = 1:size(theta_tmp, 2)
        show(model.robot, theta_tmp(:, t), 'PreservePlot', false, 'Frames', 'on');
        writeVideo(v, getframe(gcf));
    end
end
close(v);
end

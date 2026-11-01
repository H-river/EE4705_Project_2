function animateTrajectory(model, theta, videoFile, frameRate)
% ANIMATETRAJECTORY  Play the planned trajectory in the current figure; optionally
% write it to a video (videoFile = '' -> no video). Merges the two original blocks
% "Record planned trajectory to video files" and "Show the planned trajectory".
if nargin < 3, videoFile = ''; end
if nargin < 4, frameRate = 2; end

writeVid = ~isempty(videoFile);
if writeVid
    v = VideoWriter(videoFile);
    v.FrameRate = frameRate;
    open(v);
end

for t = 1:size(theta, 2)
    show(model.robot, theta(:, t), 'PreservePlot', false, 'Frames', 'on');
    drawnow;
    if writeVid
        writeVideo(v, getframe(gcf));
    end
    pause(5/20);
end

if writeVid
    close(v);
end
end

function [X, T] = updateJointsWorldPosition(model, theta)
% UPDATEJOINTSWORLDPOSITION  Forward kinematics of all frames used by the planner.
%
% INPUT
%   model : from buildRobotModel (uses .fkMode, .fkBodyNames, .robot, .S, .Mhome)
%   theta : nJoints x 1 joint vector
% OUTPUT (shared interface -- stompTrajCost / Task 5 rely on this)
%   X : nFK x 4, row k = homogeneous position [x y z 1] of frame k in the BASE frame
%       rows 1..nJoints = joint frames, row nFK = end-effector
%   T : 1 x nFK cell, T{k} = 4x4 pose of frame k in the BASE frame; T{end} = EE pose
%
% Interface change vs. original: takes `model` instead of `robot_struct`, so the same
% call works for both getTransform (Task 1/2) and PoE (Task 3+).

nFK = model.nFK;
X = zeros(nFK, 4);
T = cell(1, nFK);

switch model.fkMode
    case 'getTransform'
        for k = 1:nFK
            % Pose of body model.fkBodyNames{k} w.r.t. the base frame. model.robot is
            % DataFormat 'column', so getTransform takes `theta` directly -- the original
            % sample code built a struct configuration on every call, which was a large
            % part of its run time (efficiency fix).
            T{k} = getTransform(model.robot ,theta, model.fkBodyNames{k});          
            X(k, :) = T{k}(:, 4)'; 
        end

    case 'PoE'
        % One running product of exponentials gives every frame (see fkPoE).
        T = fkPoE(model.S, model.Mhome, theta);
        for k = 1:nFK
            X(k, :) = T{k}(:, 4)';
        end

    otherwise
        error('updateJointsWorldPosition: unknown fkMode "%s".', model.fkMode);
end
end

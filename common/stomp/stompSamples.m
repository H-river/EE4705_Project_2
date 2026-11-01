% Input:
%   nSamplePaths: the number of sample paths
%   sigma: sample covariance matrix, (nDiscretize-2) x (nDiscretize-2) (interior points only)
%   theta: mean trajectory from last iteration, nJoints x nDiscretize
% Output:
%   theta_paths: 1 x nSamplePaths cell, sampled trajectories
%   em: 1 x nJoints cell, each nSamplePaths x nDiscretize, zero-mean noise per joint

function [theta_paths, em] = stompSamples(nSamplePaths, sigma, theta)
% Sample theta (joints angles) trajectory

[nJoints, nDiscretize] = size(theta);

em = cell(1, nJoints);
ek = cell(1, nSamplePaths);

theta_paths = cell(1, nSamplePaths);
mu = zeros(1, length(sigma));

for m = 1 : nJoints
    %% TODO[T1-4] (Owner: T12) Independent sampling for each joint
    % - Each joint is sampled independently from N(mu, sigma) (mvnrnd,
    %   Statistics & ML Toolbox) -> nSamplePaths x (nDiscretize-2).
    % - Start q0 and goal qT are fixed: pad a zero column on both sides so
    %   em{m} is nSamplePaths x nDiscretize.
    em{m} = zeros(nSamplePaths, nDiscretize);   % placeholder
end

%% Regroup it by samples (unchanged)
emk = [em{:}];
for k = 1:nSamplePaths
    ek{k} = reshape(emk(k, :), nDiscretize, nJoints)';
    theta_paths{k} = theta + ek{k};
end
end

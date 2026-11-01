function [R, sigma, M] = stompSmoothingMatrices(nDiscretize, sigmaScale)
% STOMPSMOOTHINGMATRICES  Pre-computed matrices of STOMP (moved out of helperSTOMP).
%   R     : (N-2)x(N-2) control-cost matrix A'A (finite-difference acceleration)
%   sigma : sampling covariance = sigmaScale * R^-1 / sum(R^-1), keeps samples inside the voxel world
%   M     : smoothing matrix, R^-1 normalised column-wise and scaled by 1/N
if nargin < 2, sigmaScale = 1.5; end

A_k = eye(nDiscretize - 1, nDiscretize - 1);
A = -2 * eye(nDiscretize, nDiscretize);
A(1:nDiscretize - 1, 2:nDiscretize) = A(1:nDiscretize - 1, 2:nDiscretize) + A_k;
A(2:nDiscretize, 1:nDiscretize - 1) = A(2:nDiscretize, 1:nDiscretize - 1) + A_k;
A = A(:, 2:end-1);
R = A' * A;
Rinv = inv(R);
% The smoothing matrix M, normalized from Rinv by each column, no longer symmetric
M = 1 / nDiscretize * Rinv ./ max(Rinv, [], 1);
% R inverse is normalized so that the exploration is controlled to have samples within the created voxel world
sigma = sigmaScale * Rinv / sum(sum(Rinv));
end

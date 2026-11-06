% dtheta: estimated gradient, nJoints x nDiscretize
% trajProb: nSamples x nDiscretize, per-time-step probability of each sample (Eq. 11)
% em: 1 by nJoints cell, each cell is nSamples by nDiscretize matrix (sampled noise)
function dtheta = stompDTheta(trajProb, em)

nJoints = length(em);
nDiscretize = size(trajProb, 2);

% variable declaration
dtheta = zeros(nJoints, nDiscretize);

%% Probability-weighted update for every joint (STOMP Table 1, step 3):
% delta_theta_i = sum_k P(theta_k, i) * eps_k,i     (element-wise over time step i)
for m = 1:nJoints
    dtheta(m, :) = sum( trajProb .* em{m}, 1);
end
end

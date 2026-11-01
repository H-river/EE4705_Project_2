function T = fkPoE(S, Mhome, theta)
% FKPOE  Product-of-Exponentials forward kinematics for ALL planner frames at once.
%
%   T{k} = e^[S_1]th_1 * ... * e^[S_k]th_k * Mhome(:,:,k)     k = 1..nJoints
%   T{nFK} = e^[S_1]th_1 * ... * e^[S_n]th_n * Mhome(:,:,nFK) (end-effector)
%
% INPUT  S (6 x n), Mhome (4 x 4 x nFK), theta (n x 1)
% OUTPUT T (1 x nFK cell of 4x4)
%
% Owner: Jiamo (T34)
%
% Efficiency note (handout asks for it): FKinSpace from Modern Robotics returns only the
% EE pose; calling it once per joint frame is O(n^2) matrix products plus a matrix
% exponential each time. Here the running product is reused, so all frames cost O(n),
% and each exponential uses the closed-form Rodrigues formula instead of expm/MatrixExp6.

n   = size(S, 2);
nFK = size(Mhome, 3);
T   = cell(1, nFK);
for k = 1:nFK
    T{k} = eye(4);      % placeholder so the pipeline still runs
end

%% TODO[T3-2] (Owner: Jiamo) Running product over joints.
% E = eye(4);
% for i = 1:n
%     E = E * expTwist(S(:, i), theta(i));
%     T{i} = E * Mhome(:, :, i);
% end
% T{nFK} = E * Mhome(:, :, nFK);
% (Delete the placeholder loop above once this is in.)
end

function E = expTwist(Si, th) %#ok<DEFNU>
% EXPTWIST  e^([S] th) for a unit screw axis S = [w; v], closed form.
%% TODO[T3-2b] (Owner: Jiamo)
% revolute (||w|| = 1):
%   W = skew(w);  R = I + sin(th) W + (1 - cos(th)) W^2
%   p = (I th + (1 - cos(th)) W + (th - sin(th)) W^2) v
% prismatic (w = 0): R = I, p = v th
E = eye(4);
end

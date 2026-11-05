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

E = eye(4);
for i = 1:n
    E = E * expTwist(S(:, i), theta(i));
    T{i} = E * Mhome(:, :, i);
end
for k = n+1:nFK          % end-effector (and any further fixed frames)
    T{k} = E * Mhome(:, :, k);
end
end

function E = expTwist(Si, th)
% EXPTWIST  e^([S] th) for a unit screw axis S = [w; v], closed form (Lynch & Park 3.51/3.88).
% revolute (||w|| = 1):
%   R = I + sin(th) [w] + (1 - cos(th)) [w]^2
%   p = (I th + (1 - cos(th)) [w] + (th - sin(th)) [w]^2) v
% prismatic (w = 0): R = I, p = v th
w = Si(1:3);
v = Si(4:6);
if w(1) == 0 && w(2) == 0 && w(3) == 0
    E = [eye(3), v * th; 0 0 0 1];
    return
end
W  = [    0, -w(3),  w(2);
       w(3),     0, -w(1);
      -w(2),  w(1),     0];
W2 = W * W;
s  = sin(th);
c  = 1 - cos(th);
R  = eye(3) + s * W + c * W2;
p  = (th * eye(3) + c * W + (th - s) * W2) * v;
E  = [R, p; 0 0 0 1];
end

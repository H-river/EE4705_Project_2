function tbl = screwAxisTable(model, csvFile)
% SCREWAXISTABLE  Report material for Task 3.
%   * Table of omega_i, q_i, v_i of every joint (method 1, computeScrewAxes), next to the
%     URDF joint <origin>/<axis> data (Joint.JointToParentTransform / JointAxis), so the
%     report can relate S_i to the robot description.
%   * Method-2 cross-check (matrix log): with only joint i moved by a,
%         T_i(a e_i) = e^([S_i] a) M_i   ->   [S_i] a = logm(T_i(a e_i) M_i^-1)
%     The column err2 = ||S_i(method 1) - S_i(method 2)|| must be ~1e-12.
%
% INPUT  model   : from buildRobotModel (any fkMode; S is recomputed here)
%        csvFile : (optional) write the table to this CSV ('' = do not write)
% OUTPUT tbl     : MATLAB table, one row per joint
%
% Owner: Jiamo (T34)

if nargin < 2, csvFile = ''; end

[S, Mhome] = computeScrewAxes(model);
n = model.nJoints;
z = zeros(n, 1);
a = 0.7;                                   % test displacement for method 2 [rad or m]

joint = strings(n, 1);  type = strings(n, 1);
omega = zeros(n, 3);  qpt = zeros(n, 3);  v = zeros(n, 3);
urdfXYZ = zeros(n, 3);  urdfAxis = zeros(n, 3);  err2 = zeros(n, 1);

for i = 1:n
    jnt = model.robot.getBody(model.jointBodyNames{i}).Joint;
    joint(i) = string(jnt.Name);
    type(i)  = string(jnt.Type);
    omega(i, :) = S(1:3, i)';
    v(i, :)     = S(4:6, i)';
    if any(omega(i, :))
        % point on the axis closest to the base origin: q = omega x v (since v = -omega x q)
        qpt(i, :) = cross(omega(i, :), v(i, :));
    end
    urdfXYZ(i, :)  = jnt.JointToParentTransform(1:3, 4)';
    urdfAxis(i, :) = jnt.JointAxis;

    % ---- method 2: matrix logarithm ----
    th = z;  th(i) = a;
    Ti = getTransform(model.robot, th, model.fkBodyNames{i});
    L  = real(logm(Ti / Mhome(:, :, i))) / a;          % = [S_i] (se(3) matrix)
    S2 = [L(3, 2); L(1, 3); L(2, 1); L(1:3, 4)];
    if norm(S2(1:3)) > 1e-9, S2 = S2 / norm(S2(1:3)); else, S2 = S2 / norm(S2(4:6)); end
    err2(i) = norm(S2 - S(:, i));
end

tbl = table(joint, type, omega, qpt, v, urdfXYZ, urdfAxis, err2, ...
    'VariableNames', {'joint', 'type', 'omega', 'q_on_axis', 'v', ...
                      'urdf_origin_xyz', 'urdf_axis', 'err_vs_matrixLog'});

fprintf('\nScrew axes of %s (space frame, home = zero vector):\n', model.name);
fprintf('%-24s %-28s %-28s %-28s %9s\n', 'joint', 'omega', 'q (on axis)', 'v', 'err(log)');
for i = 1:n
    fprintf('%-24s %-28s %-28s %-28s %9.1e\n', joint(i), vecStr(omega(i, :)), ...
        vecStr(qpt(i, :)), vecStr(v(i, :)), err2(i));
end
fprintf('Max difference method 1 vs method 2 (matrix log): %.2e\n', max(err2));

if ~isempty(csvFile)
    writetable(splitvars(tbl), csvFile);
end
end

function s = vecStr(x)
x(abs(x) < 5e-13) = 0;
s = sprintf('[%7.4f %7.4f %7.4f]', x);
end

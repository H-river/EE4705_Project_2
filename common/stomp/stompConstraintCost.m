function c = stompConstraintCost(T_ee, cfg)
% STOMPCONSTRAINTCOST  Task-5 end-effector orientation cost of ONE waypoint.
%
% Keeps a chosen EE body axis (default: y-axis) aligned with a chosen world axis
% (default: world z, i.e. "upright" -- think of carrying a cup of coffee).
%
% INPUT  T_ee : 4x4 EE pose in the base/world frame (T{end} from updateJointsWorldPosition)
%        cfg.eeAxis         : 1/2/3 = EE x/y/z axis to constrain          (default 2)
%        cfg.worldAxis      : 3x1 target direction in the world frame     (default [0;0;1])
%        cfg.constraintType : 'l1' (default) | 'l2sq' | 'angle'          (optional, for the report)
% OUTPUT c    : scalar >= 0, equal to 0 when the axis is perfectly aligned
%
% Column k of the rotation block R = T_ee(1:3,1:3) is the EE body axis k (x, y, z)
% expressed in the world frame, so
%   a = T_ee(1:3, eeAxis)   is the constrained EE axis in the world frame (unit vector).
%
% Cost options (all are 0 at perfect alignment and grow with the deviation):
%   'l1'    c = || a - worldAxis ||_1              handout Section 1.1, option 2 (default)
%                                                  range [0, 2] for worldAxis = [0;0;1]
%   'l2sq'  c = || a - worldAxis ||_2^2            = 2 (1 - cos(angle)), smooth at 0
%   'angle' c = atan2(||a x w||, a'w)  [rad]       the actual deviation angle
%
% Why no toolbox call in here (e.g. tform2rotm): this function is called once per
% waypoint, per noisy sample, per STOMP iteration (nPaths * nDiscretize * nIter, i.e.
% tens of thousands of calls per run). tform2rotm validates its input every call;
% plain indexing T_ee(1:3,1:3) does the same job with no overhead and no dependency.
% The toolbox helpers (tform2rotm, tform2trvec, getTransform) are used in the
% analysis / plotting code (plotEEAxisDeviation, plotConfigOverlay, main_task5), which
% runs once per figure -- there the readability is worth the overhead.
%
% Note: this only constrains the DIRECTION of one axis. The EE may still rotate freely
% about that axis (a cup stays upright however it is spun). Penalising distance to a
% full target rotation (handout option 1) would be stricter.
%
% Owner: T5

%% Options with defaults, so the function also works with a minimal cfg
axisIdx   = getOpt(cfg, 'eeAxis',         2);
worldAxis = getOpt(cfg, 'worldAxis',      [0; 0; 1]);
mode      = getOpt(cfg, 'constraintType', 'l1');

%% Cheap input checks (a silent wrong axis would just give a plausible-looking cost)
if ~isequal(size(T_ee), [4 4])
    error('stompConstraintCost: T_ee must be a 4x4 pose, got %s.', mat2str(size(T_ee)));
end
if ~isscalar(axisIdx) || ~any(axisIdx == [1 2 3])
    error('stompConstraintCost: cfg.eeAxis must be 1, 2 or 3.');
end
worldAxis = worldAxis(:);
nW = norm(worldAxis);
if numel(worldAxis) ~= 3 || nW == 0
    error('stompConstraintCost: cfg.worldAxis must be a non-zero 3-vector.');
end
worldAxis = worldAxis / nW;                  % make sure it is a unit vector

%% Constrained EE axis in the world frame
a = T_ee(1:3, axisIdx);
a = a / norm(a);                             % guards against round-off drift in R

%% Orientation penalty
switch lower(mode)
    case 'l1'
        c = norm(a - worldAxis, 1);
    case 'l2sq'
        c = sum((a - worldAxis).^2);
    case 'angle'
        % atan2 form: accurate for tiny angles, no clamping, never complex
        % (same formula as plotEEAxisDeviation, so cost and plot agree).
        c = atan2(norm(cross(a, worldAxis)), dot(a, worldAxis));
    otherwise
        error('stompConstraintCost: unknown cfg.constraintType "%s" (use l1 | l2sq | angle).', mode);
end
end


function v = getOpt(s, name, default)
% GETOPT  s.(name) if present and non-empty, otherwise the default.
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end

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
%   y_sb = T_ee(1:3, 2)   is the EE y-axis in the world frame (unit vector).
%
% Cost options (all are 0 at perfect alignment and grow with the deviation):
%   'l1'    c = || y_sb - worldAxis ||_1           handout Section 1.1, option 2 (default)
%                                                  range [0, 2] for worldAxis = [0;0;1]
%   'l2sq'  c = || y_sb - worldAxis ||_2^2         = 2 (1 - cos(angle)), smooth at 0
%   'angle' c = acos( y_sb' * worldAxis )  [rad]   the actual deviation angle
%
% Note: this only constrains the DIRECTION of one axis. The EE may still rotate freely
% about that axis (a cup stays upright however it is spun). Penalising distance to a
% full target rotation (handout option 1) would be stricter.
%
% Owner: T5

%% Defaults, so the function also works with a minimal cfg
axisIdx = 2;
if isfield(cfg, 'eeAxis') && ~isempty(cfg.eeAxis)
    axisIdx = cfg.eeAxis;
end

worldAxis = [0; 0; 1];
if isfield(cfg, 'worldAxis') && ~isempty(cfg.worldAxis)
    worldAxis = cfg.worldAxis(:);
end
worldAxis = worldAxis / norm(worldAxis);     % make sure it is a unit vector

mode = 'l1';
if isfield(cfg, 'constraintType') && ~isempty(cfg.constraintType)
    mode = cfg.constraintType;
end

%% Constrained EE axis in the world frame
y_sb = T_ee(1:3, axisIdx);

%% Orientation penalty
switch lower(mode)
    case 'l1'
        c = norm(y_sb - worldAxis, 1);
    case 'l2sq'
        c = sum((y_sb - worldAxis).^2);
    case 'angle'
        % clamp against round-off so acos never returns a complex number
        c = acos(max(-1, min(1, y_sb' * worldAxis)));
    otherwise
        error('stompConstraintCost: unknown cfg.constraintType "%s" (use l1 | l2sq | angle).', mode);
end
end

function c = stompConstraintCost(T_ee, cfg)
% STOMPCONSTRAINTCOST  Task-5 end-effector orientation cost of ONE waypoint.
% INPUT  T_ee : 4x4 EE pose in the base frame (T{end} from updateJointsWorldPosition)
%        cfg.eeAxis    : 1/2/3 = EE x/y/z axis to constrain
%        cfg.worldAxis : 3x1 target direction in the world frame (default [0;0;1])
% OUTPUT c    : scalar >= 0, 0 when the axis is perfectly aligned
%
% Owner: T5

c = 0;   % placeholder

%% TODO[T5-1] (Owner: T5) Orientation penalty (handout Section 1.1, option 2):
%   y_sb = T_ee(1:3, cfg.eeAxis);   c = norm(y_sb - cfg.worldAxis, 1);
% Alternatives worth comparing in the report: squared l2 norm, or the angle
% acos(y_sb' * worldAxis). Option 1 (penalise distance to a full target rotation)
% is stricter -- also constrains rotation about the upright axis.
end

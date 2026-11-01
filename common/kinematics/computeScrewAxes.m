function [S, Mhome] = computeScrewAxes(model)
% COMPUTESCREWAXES  Space-frame screw axes and home poses for PoE forward kinematics.
% Called ONCE in buildRobotModel (screw axes depend only on the home configuration).
%
% OUTPUT
%   S     : 6 x nJoints, column i = S_i = [omega_i; v_i] in the base/space frame
%   Mhome : 4 x 4 x nFK, Mhome(:,:,k) = pose of model.fkBodyNames{k} at theta = 0
%           (k = 1..nJoints joint frames, k = nFK the end-effector)
%
% Owner: Jiamo (T34)

n   = model.nJoints;
nFK = model.nFK;
S     = zeros(6, n);
Mhome = repmat(eye(4), 1, 1, nFK);   % placeholder so the pipeline still runs

%% TODO[T3-1] (Owner: Jiamo) Determine S_i for every joint.
% Method 1 (planned for the report -- "planar car" cross-product method):
%   * Use the ZERO vector as the home configuration, NOT homeConfiguration(robot):
%     PoE angles are measured from the pose in which Mhome is taken. (For most robots
%     they coincide, but check -- some robots' home config is not all zeros.)
%   * For joint i: Ti = getTransform(model.robot, zeros(n,1), model.jointBodyNames{i});
%       axis_local = model.robot.getBody(name).Joint.JointAxis  (in the JOINT frame)
%       Check Joint.ChildToJointTransform is identity, otherwise the joint frame is not
%       the body frame and axis_local must be rotated by it as well.
%       omega_i = Ti(1:3,1:3) * axis_local(:);   q_i = Ti(1:3,4);
%       revolute / continuous: S_i = [omega_i; -cross(omega_i, q_i)]
%       prismatic:             S_i = [0; 0; 0; omega_i]
%   * Mhome(:,:,k) = getTransform at zeros(n,1) for every model.fkBodyNames{k}.
%
%% TODO[T3-1b] (Owner: Jiamo, optional) Method 2 cross-check (matrix-log method):
%   T(theta_i = a) * inv(T(theta_i = 0)) = expm([S_i] a)  ->  S_i = logm(.)/a,
%   normalise so ||omega|| = 1 (revolute) or ||v|| = 1 (prismatic); compare with method 1.
%
%% TODO[T3-1c] (Owner: Jiamo) Report material: print a table of omega_i, q_i, v_i
%   for the chosen robot and relate them to the URDF <origin>/<axis> tags.
end

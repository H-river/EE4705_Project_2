function [S, Mhome] = computeScrewAxes(model)
% COMPUTESCREWAXES  Space-frame screw axes and home poses for PoE forward kinematics.
% Called ONCE in buildRobotModel (screw axes depend only on the home configuration).
%
% OUTPUT
%   S     : 6 x nJoints, column i = S_i = [omega_i; v_i] in the base/space frame
%   Mhome : 4 x 4 x nFK, Mhome(:,:,k) = pose of model.fkBodyNames{k} at theta = 0
%           (k = 1..nJoints joint frames, k = nFK the end-effector)
%
% Method 1 ("cross-product" method, Lynch & Park Ch. 4.1):
%   * Home configuration = the ZERO vector, not homeConfiguration(robot). PoE angles are
%     measured from the pose in which Mhome is taken. (For the Panda the zero vector is
%     even outside the limits of joints 4 and 6 -- that does not matter for FK, it is
%     just the reference pose.)
%   * A rigidBodyTree body pose is
%         T_body = T_parent * JointToParentTransform * Tjoint(q) * ChildToJointTransform
%     and the joint axis (Joint.JointAxis) is expressed in the JOINT frame, i.e. the frame
%     T_body * inv(ChildToJointTransform). For URDF-imported robots ChildToJoint is the
%     identity, but it is handled in general here instead of being assumed.
%   * revolute / continuous: S_i = [omega_i; -omega_i x q_i]   (q_i = a point on the axis)
%     prismatic:             S_i = [0; 0; 0; omega_i]
%
% Method 2 (matrix-log cross-check) and the report table: Task3_PoE/screwAxisTable.m
%
% Owner: Jiamo (T34)

n   = model.nJoints;
nFK = model.nFK;
z   = zeros(n, 1);

S     = zeros(6, n);
Mhome = zeros(4, 4, nFK);

for i = 1:n
    name = model.jointBodyNames{i};
    jnt  = model.robot.getBody(name).Joint;

    Tbody  = getTransform(model.robot, z, name);
    Tjoint = Tbody / jnt.ChildToJointTransform;      % joint frame in the space frame

    omega = Tjoint(1:3, 1:3) * jnt.JointAxis(:);
    omega = omega / norm(omega);
    q     = Tjoint(1:3, 4);

    switch jnt.Type
        case {'revolute', 'continuous'}
            S(:, i) = [omega; -cross(omega, q)];
        case 'prismatic'
            S(:, i) = [zeros(3, 1); omega];
        otherwise
            error('computeScrewAxes: unsupported joint type "%s" (%s).', jnt.Type, name);
    end
end

for k = 1:nFK
    Mhome(:, :, k) = getTransform(model.robot, z, model.fkBodyNames{k});
end
end

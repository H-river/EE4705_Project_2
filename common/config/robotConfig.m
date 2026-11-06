function rc = robotConfig()
% ROBOTCONFIG  Single source of truth for "the robot we use from Task 2 onwards".
% Tasks 2-5 all call this, so changing the robot here switches every task.
%
% Task-2 robot: Franka Emika Panda (built-in loadrobot model, 7 revolute joints).
% Different from the Kinova: no continuous joints, coincident frames (link1/link2,
% link5/link6), elbow offsets, and a gripper whose two prismatic finger joints are
% frozen by buildRobotModel. The TCP is a virtual body 0.1034 m along panda_hand's
% z axis (between the fingertips), added by buildRobotModel.
% isDecided = false would make scene_newrobot_base fall back to the Kinova scene.
rc.robotName  = 'frankaEmikaPanda';
rc.eeName     = 'panda_tcp';
rc.tcpParent  = 'panda_hand';
rc.tcpOffset  = [0 0 0.1034];
rc.isDecided  = true;

%% Collision spheres: one radius per segment (frame k-1 -> frame k, see stompRobotSphere),
% fitted to the Panda collision meshes by Task2_NewRobot/fitSphereRadii (99th percentile
% of the vertex-to-segment distance over 200 random configurations, rounded up to mm).
% The first version used 0.05 m for every link: the plot task2_spheres.png showed those
% spheres inside the link meshes (median mesh distance 8-10 cm), so only the 5 cm
% safety margin kept the arm away from obstacles. With fitted radii the margin can be a
% real margin again (2 cm).
%   segment ends:   link1  link2  link3  link4  link5  link6  link7  tcp
rc.sphereRadius = [0.155  0.155  0.130  0.112  0.130  0.130  0.130  0.103];
rc.safetyMargin = 0.02;     % epsilon in Eq. (13) for this robot (Kinova / Task 1: 0.05)
end

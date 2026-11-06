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

%% Collision-sphere radius
% Scalar = same radius for every link; vector (1 x nFK) = one radius per link
% (row k covers the segment from frame k-1 to frame k, see stompRobotSphere).
% Coverage is checked in Task2_NewRobot/main_task2.m (task2_spheres.png).
rc.sphereRadius = 0.05;
end

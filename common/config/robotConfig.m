function rc = robotConfig()
% ROBOTCONFIG  Single source of truth for "the robot we use from Task 2 onwards".
% Tasks 2-5 all call this, so changing the robot here switches every task.
%
% Until Task 2 is decided this returns Kinova Gen3, so Tasks 3/4/5 can be
% developed in parallel on a robot that is already known to work.

%% TODO[T2-1] (Owner: Joshua Lim) Choose the Task-2 robot and fill in the fields below.
% - Built-in robots: see `help loadrobot` (e.g. 'universalUR5e', 'frankaEmikaPanda',
%   'abbIrb120', 'kukaIiwa7', ...). Pick one whose geometry differs from Kinova
%   (prismatic joint / fixed base links / different DOF are all "interesting").
% - External URDF: set rc.robotName to the .urdf path; buildRobotModel will call
%   importrobot. You also need the mesh path for visualisation (see handout Task 2).
% - rc.eeName must be the body that represents the tool / end-effector.
% - After choosing, set rc.isDecided = true and tell T34 + T5 (screw axes and
%   EE constraint depend on it).
rc.robotName  = 'frankaEmikaPanda';
rc.eeName     = 'panda_tcp';
rc.tcpParent  = 'panda_hand'; 
rc.tcpOffset  = [0 0 0.1034];
rc.isDecided  = true;

%% TODO[T2-2] (Owner: Joshua Lim) Collision-sphere radius for the chosen robot.
% Scalar = same radius for every link; vector (1 x nFK) = one radius per link
% (row k covers the segment from frame k-1 to frame k, see stompRobotSphere).
% Check visually with `show(robot)` that the spheres cover the links.
rc.sphereRadius = 0.05;
end

# EE4705 Project 2 Group 9: speaker notes

## Slide 1

JOSHUA, 10 seconds. Good afternoon. We are Group 9. Our project plans collision-free paths for robot arms with STOMP, on two different manipulators. I will cover the planner and the first two tasks, Jiamo covers kinematics, custom scenes and our improvements, and Dexter covers the end-effector constraint.

## Slide 2

JOSHUA, 15 seconds. All five tasks share one code base. One thing applies to every number today: a result only counts if MATLAB’s mesh-based collision check passes, both at the waypoints and at sub-steps between them.

## Slide 3

JOSHUA, 30 seconds. STOMP optimises the whole joint trajectory, 20 waypoints with the start and goal fixed. Each iteration draws 20 noisy trajectories. The noise comes from the inverse of the acceleration matrix R, so it is smooth. We score each sample at each waypoint, turn the costs into probabilities per waypoint, and update with the probability-weighted noise, smoothed by M. The obstacle cost is high when a sphere is near an obstacle and moving fast, and we never need its gradient.

## Slide 4

JOSHUA, 25 seconds. We completed the five unfinished functions and kept the original scene. The straight-line start collides at 8 of the 20 waypoints; after 23 iterations the path is collision-free. Along the way we found ten bugs in the sample code. The worst: a sample leaving the voxel world got zero cost, so it looked like the best sample. Also, the control cost added meaningless cross-joint terms, about 45 percent too high on this trajectory, and the last 6 centimetres of the arm had no collision spheres.

## Slide 5

JOSHUA, 30 seconds. We chose the Franka Emika Panda: 7 revolute joints, but tight joint limits, link offsets and a gripper. We froze the two finger joints and added a tool frame at the fingertips, so poses mean the grasp point. With the provided IK settings the solver missed the goal by 8.9 centimetres, so we enabled random restarts under a fixed seed, which reaches it exactly. The Task 1 scene was trivial on this robot, because a straight line in joint space is curved in Cartesian space, so we placed the box on the robot’s actual path. 13 iterations to a collision-free path.

## Slide 6

JOSHUA, 15 seconds. Our first 5 centimetre spheres sat inside the links, so only the safety margin kept the arm clear. We fitted one radius per link from the collision meshes: the 99th percentile of the vertex distance over 200 random configurations. That lets the 5 centimetre margin shrink to a true 2 centimetre margin.

## Slide 7

JOSHUA, 20 seconds, then hand over. The provided planner stops after a single small change in cost. Because STOMP is stochastic, that can happen while the path still collides. We require three in a row. Over 7 scenes and 5 seeds, collision-free runs go from 29 to all 35; the cost is more iterations. Jiamo will now take over for kinematics.

## Slide 8

JIAMO, 30 seconds. Task 3 replaces getTransform with the Product of Exponentials. Each joint has a screw axis S, made of omega and v. Omega is the joint axis in the base frame: the URDF gives it in the joint’s own frame, so we rotate it with the joint pose. q is the joint origin, and v is minus omega cross q, the same construction as the planar car example from class. We also recovered each axis with the matrix logarithm, and the two methods agree to 4e-16. The axes are computed once per robot.

## Slide 9

JIAMO, 25 seconds. Accuracy first: over 500 random poses, every frame matches getTransform to about 1e-15. The planner needs all joint frames for the collision spheres, so we take one running product from the base and read each frame along the way, using the closed-form exponential from the assignment instead of expm. That is 35 times faster, and a STOMP iteration drops from 527 to 38 milliseconds. With the same seed both versions produce the identical trajectory. This speed is what makes our multi-seed studies affordable.

## Slide 10

JIAMO, 25 seconds. We extended the voxel world to rotated boxes, spheres and cylinders. The three scenes are designed so the straight-line start collides. In the first, the arm must lift over a wall, with a post blocking the short way round. In the second it reaches under a shelf and turns the tool horizontal. The third is a large re-orientation past a column, a ball and a rotated panel. Each scene succeeded for all 10 random seeds.

## Slide 11

JIAMO, 15 seconds. This is scene 3, one frame per STOMP iteration. The dotted line is the initial guess, the black line is the current path, and the title turns green once the path is collision-free.

## Slide 12

JIAMO, 35 seconds. We measured each change on seven scenes with five seeds. Plain STOMP ignores joint limits, and our plans exceeded the Panda’s limits by up to 0.52 radians, about 30 degrees. We project every sample and every update back into the limits. That removes the violations, but on its own it makes the tightest scenes harder: 31 of 35. So we added sample reuse: the five cheapest samples from the previous iteration join the next update, and because their costs are already known it costs nothing extra. Now all 35 runs succeed. We also tried shrinking the noise over time. That made it worse, 29 of 35, so we left it off and report it as a negative result.

## Slide 13

JIAMO, 25 seconds, then hand over. Three practical additions. A GUI lets you choose a scene, the kinematics and the planner options, then plan and rotate the view. Second, interpolating continuous joints along the shortest angle shortens the end-effector path in our Kinova demo from 3.31 to 0.79 metres. Third, verification: a path counts as collision-free only if the mesh check passes at the waypoints and at nine steps between each pair. Dexter will now present Task 5.

## Slide 14

DEXTER, 30 seconds. Task 5 is like carrying a cup of coffee: the gripper’s y-axis has to stay vertical while the arm moves. We use option 2 from the handout. At each waypoint the constraint cost is the l1 norm of the difference between the gripper’s y-axis and the world z-axis. The y-axis is simply a column of the rotation matrix, so forward kinematics already gives it for free. The cost is added to the local cost with a weight, so samples that stay upright get a higher probability. Only this one axis is constrained: the gripper can still spin about the vertical.

## Slide 15

DEXTER, 30 seconds. As the handout suggests, the start and goal are themselves upright: the tool points radially outward with its y-axis vertical, half a metre from the base axis. A 16 centimetre box sits on the arc between them, so 6 of the 20 straight-line waypoints collide. Our first version used a height of 0.30 metres. The Panda cannot reach that orientation there: IK stopped 2.8 centimetres away with joint 5 at its limit, even with random restarts. At 0.20 metres both poses are reached exactly, so we re-tuned the scene and lowered the box with them.

## Slide 16

DEXTER, 40 seconds. Here is the same scene and seed, planned without the constraint on the left and with it on the right. The orange arrow is the y-axis: on the left it tilts by up to 36.8 degrees, on the right it stays within 1.9 degrees. Both paths are collision-free. The price is more iterations, 27 to 124, because the planner must find a detour that is also upright. The constraint is a soft cost, so the tilt is small but not exactly zero.

## Slide 17

DEXTER, 25 seconds. This plot gives the tilt at every waypoint. Without the constraint it rises smoothly to 36.8 degrees in the middle of the detour. With the constraint it stays below 2 degrees the whole way. The side views show the path and the orange y-axis arrows. The end-effector path length hardly changes, 0.84 versus 0.82 metres. Each iteration also takes about the same time, roughly 37 milliseconds, so the extra planning time comes from needing more iterations, not from computing the constraint.

## Slide 18

DEXTER, 25 seconds. One run per weight can mislead, so each weight was run with five seeds. Our rule picks the smallest weight that is collision-free on every seed and keeps the median worst-case tilt under 5 degrees, which is 10. At 1000 and above the constraint overrides obstacle avoidance and no run is collision-free.

## Slide 19

DEXTER, 20 seconds. To summarise: all five tasks are solved, and every result passes a mesh-based check at and between the waypoints. We also want to be clear about the limits: the spheres hold 99 percent of the mesh, STOMP is a local stochastic optimiser, and the upright constraint is a soft cost. Thank you, we are happy to take questions.

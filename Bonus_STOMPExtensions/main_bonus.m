%% Bonus -- extensions of STOMP and the joint-angle discontinuity
% Owner: Jiamo
%
% The extensions live in common/stomp/runSTOMP.m and common/stomp/initTrajectory.m and are
% switched by cfg (common/config/defaultConfig.m):
%   cfg.shortestAngleInit  (default true)  continuous joints interpolated along the
%                                          shortest angle (handout tip on +-pi)
%   cfg.enforceJointLimits (default true)  samples and updates projected into the joint
%                                          limits -- the original STOMP ignores them
%   cfg.nReuse             (default 0)     reuse the cheapest samples of the last iteration
%   cfg.noiseDecay         (default 1)     shrink the exploration noise over the iterations
%
%   Part A  demoAngleWrap : Kinova, base joint 2.8 -> -2.8 rad, naive vs shortest-angle init
%   Part B  stompAblation : every extension on every project scene, several seeds
%   Part C  compare_patience_all : convergence patience P = 1 vs P = 3 on every scene
% Outputs: Bonus_STOMPExtensions/results/ (CSV tables + figures). Run time ~15 min,
% most of it Parts B and C (set nSeeds smaller for a quick look at Part B).

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisDir, '..', 'common')));
addpath(thisDir);
outDir = fullfile(thisDir, 'results');
nSeeds = 5;

%% Part A: joint-angle discontinuity
wrapTbl = demoAngleWrap(outDir);

%% Part B: ablation of the STOMP extensions
close all;
ablTbl = stompAblation(outDir, 1:nSeeds);

%% Part C: convergence patience (this script clears the workspace, so it comes last)
close all;
run(fullfile(thisDir, 'compare_patience_all.m'));

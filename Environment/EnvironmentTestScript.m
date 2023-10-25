clc
clf
clear all

hold on;
% Creates environment, returns interactive objects and robots (including
% grippers)
[SBrobot, URrobot, URGripper1, URGripper2, SBGripper1, SBGripper2, nozzleObj, clothObj] = Environment.CreateEnvironment();

URrobot.model.teach();

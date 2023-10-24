clc
clf
clear all

hold on;
[robot1, robot2] = Environment.CreateEnvironment()

robot2.model.teach();
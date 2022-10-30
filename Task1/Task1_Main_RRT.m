clc ;
clear;
close all;
q_init = [0;0;0;0];              % configuration init
q_goal = [2;2.1;1.9;1.7];
% q_goal = [q_goal(1);q_goal(1)+q_goal(2);q_goal(1)+q_goal(2)+q_goal(3);q_goal(1)+q_goal(2)+q_goal(3)+q_goal(4)];
NumNodes = 5e5;
del_q = 0.05;                       % i.e 1/20 
L = 8;      % length of one section of manipulator

%%%%%% set obs in worksapce
Obs = {[5 25 25 5;20 20 35 35], [-30 -10 -10 -30;-25 -25 10 10], [-8 30 30 -8; -25 -25 -5 -5]};

[path,V,E] = serial_manipulator_RRT(q_init,q_goal,NumNodes,del_q,Obs,L);
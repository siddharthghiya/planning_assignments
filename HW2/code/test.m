clc;
startQ = [1.56566 6.03025 2.13871 3.67735 1.40625];
goalQ = [1.34002 2.40607 0.186408 2.96768 2.09464];

% % PRM
% planner_id = 3;
% fprintf("PRM\n");
% runtest('map2.txt',startQ, goalQ, planner_id);
% fprintf("\n");


%% RRT
planner_id = 0;
fprintf("RRT\n");
runtest('map2.txt',startQ, goalQ, planner_id);
fprintf("\n");

%% RRTConnect
planner_id = 1;
fprintf("RRTConnect\n");
runtest('map2.txt',startQ, goalQ, planner_id);
fprintf("\n");

%% RRTSTar
planner_id = 2;
fprintf("RRTStar\n");
runtest('map2.txt',startQ, goalQ, planner_id);
fprintf("\n");
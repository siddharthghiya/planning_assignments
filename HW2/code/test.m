startQ = [pi/2 0 0 pi/2 pi/2];
% startQ = [pi/2 pi/2 pi/2 pi/2 pi/4];
% startQ = [pi/2 pi/4 pi/2 pi/4 pi/2];
goalQ = [pi/8 3*pi/4 pi 0.9*pi 1.5*pi];
planner_id = 3;
runtest('map2.txt',startQ, goalQ, planner_id);

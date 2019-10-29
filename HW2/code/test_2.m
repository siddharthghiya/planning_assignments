%main test - RRT Star
startQ = [pi/2 pi/2 pi/2 pi+1e-1 pi pi/2];
goalQ = [pi/8 3*pi/4 pi 0.9*pi 1.5*pi pi/4];
runtest('map2.txt', startQ, goalQ, 0);

% %main test - PRM
% startQ = [pi/2 pi/2 pi/2 pi+1e-1 pi pi/2];
% goalQ = [pi/8 3*pi/4 pi 0.9*pi 1.5*pi pi/4];
% runtest('map2.txt', startQ, goalQ, 3);

% %small test - PRM
% startQ = [pi/2 2*pi/2 pi/2 pi+1e-1];
% goalQ = [pi/2 pi/2 -pi/5 5];
% runtest('map2.txt', startQ, goalQ, 3);
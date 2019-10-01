This project consists implementation of two planners for catching a moving target(with given trajectory) in a 8 connected gridworld setting.
1. Planner.cpp : This is a simple Dijkstra planner implementation which first calculates a target point in the target trajectory. This target point is the most optimal point for the robot to go and wait.
2. Planner_time.cpp : This is a 3D planner, which first calulcated a heuristic for all the points in the grid using multi goal Dijkstra and then plans in 3D.

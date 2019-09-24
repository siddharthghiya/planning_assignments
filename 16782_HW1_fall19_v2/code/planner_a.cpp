/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <mex.h>
#include <iostream>
using namespace std;
#include <queue>
#include <vector>
#include <unordered_map>

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8
#define e 10

class node{
public:
    int posX, posY;
    double h;
    double g;
    int cost;
    node* parent;
    int time_steps;
};

struct CompareG{
    bool operator()(const node* lhs, const node* rhs) const
    {
        return (lhs->g + e*lhs->h) > (rhs->g + e*rhs->h);
    }
};

double calculate_heuristic(node* nodePtr, node* goalPtr){
        int startX = nodePtr->posX;
        int startY = nodePtr->posY;
        int goalX = goalPtr->posX;
        int goalY = goalPtr->posY;
        
        return (double)sqrt(((startX-goalX)*(startX-goalX) + (startY-goalY)*(startY-goalY)));
    }

// int GETMAPINDEX(int posX, int posY, int sizeX, int sizeY){
//     int key =  posY * sizeX + posX;

//     return key;
// }

int calculate_cost(node* goalPtr, int x_size, int y_size, node* startPtr, unordered_map<int, node*> &openPtrList, unordered_map<int, bool> &closedPtrList, double* map, int collision_thresh, int curr_time){
    //important initialisations
    node* currentPtr;
    int neighborX, neighborY, neighborKey, currentKey;
    int goalKey = GETMAPINDEX(goalPtr->posX, goalPtr->posY, x_size, y_size);
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    //performance paramters
    int path_cost;
    int steps = 0;
    priority_queue<node*, vector<node*>, CompareG> open_list;
    //push the startPtr in the open list
    open_list.push(startPtr);
    unordered_map<int, bool> closed_list;

    //if goal node is closed, don't replan
    if (closedPtrList[goalKey]){
        currentPtr = openPtrList[goalKey];
        path_cost = currentPtr->cost;
        return path_cost;
    }

    // //iterate through the keys in closed lists and put them in open_list
    // for (const auto & [key,value] : closedPtrList) {    
    //     open_list.push(openPtrList[key]);   
    //     closedPtrList[key] = false;
    // }

    //A* search
    while(!open_list.empty()){
        //remove the top node from the open_list with the minimum f value and add it to the closed list.
        currentPtr = open_list.top();
        open_list.pop();
        currentKey = GETMAPINDEX(currentPtr->posX, currentPtr->posY, x_size, y_size);
        closed_list[currentKey] = true;
        closedPtrList[currentKey] = true;

        //if goal node is closed, exit the planner
        if (closed_list[goalKey]){
            break;
        }

        //go through the neighbours of the current node.
        for (int dir = 0; dir < NUMOFDIRS; dir++){
            int neighborX = currentPtr->posX + dX[dir];
            int neighborY = currentPtr->posY + dY[dir];
            
            //only if the neighbor is within bounds and within collision threshold
            if ((neighborX >= 1 && neighborX <= x_size && neighborY >= 1 && neighborY <= y_size) && ((int)map[GETMAPINDEX(neighborX,neighborY,x_size,y_size)] < collision_thresh)){

                //generate the neighbor key.
                neighborKey = GETMAPINDEX(neighborX, neighborY, x_size, y_size);
                
                //if the neighbor is already in the closed list, skip.
                if (closedPtrList[neighborKey]){
                    continue;
                }

                node* neighborPtr = NULL; 
                //check if the neighbor has been visited before
                if (openPtrList[neighborKey]){
                    neighborPtr = openPtrList[neighborKey];
                    //update the gvalue and the parent
                    if(neighborPtr->g > currentPtr->g + neighborPtr->cost){
                        neighborPtr->g = currentPtr->g + neighborPtr->cost;
                        neighborPtr->time_steps = currentPtr->time_steps + 1;
                        neighborPtr->parent = currentPtr;
                    }
                }
                else{
                    neighborPtr = new node();
                    neighborPtr->posX = neighborX;
                    neighborPtr->posY = neighborY;
                    neighborPtr->h = calculate_heuristic(neighborPtr, goalPtr);
                    neighborPtr->cost = (int)map[GETMAPINDEX(neighborX,neighborY,x_size,y_size)];
                    neighborPtr->time_steps = currentPtr->time_steps + 1;
                    neighborPtr->g = currentPtr->g + neighborPtr->cost;
                    neighborPtr->parent = currentPtr;
                    openPtrList[neighborKey] = neighborPtr;
                    open_list.push(neighborPtr);   
                }
            }
        }
    }
    path_cost = currentPtr->g;
    return path_cost;
}

static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{   
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    //static unordered_map to keep track of actions.
    static unordered_map<int,int> action_x;
    static unordered_map<int,int> action_y;
    int key;

    if(curr_time == 0){
        //Create the start_node pointer and store it's values, push it into the open_list.
        // set<node*, vector<node*>, CompareG> open_list;
        unordered_map<int, node*> openPtrList;
        unordered_map<int, bool> closedPtrList;
        node* aimPtr;
        node* tmpPtr;
        node* currentPtr;
        node* previousPtr;
        node* startPtr = new node();
        startPtr->posX = robotposeX; 
        startPtr->posY = robotposeY;
        // open_list.push(startPtr);
        openPtrList[GETMAPINDEX(startPtr->posX, startPtr->posY, x_size, y_size)] = startPtr;
        int min_cost, path_cost;
        int path_time;
        int goalKey;

        //loop thorugh all the goal positions.
        for (int t = 0; t < target_steps; t++){
            // cout << t << endl;
            node* goalPtr = new node();
            goalPtr->posX = (int) target_traj[t];
            goalPtr->posY = (int) target_traj[t+target_steps];
            goalKey = GETMAPINDEX(goalPtr->posX, goalPtr->posY, x_size, y_size);
            startPtr->h = calculate_heuristic(startPtr, goalPtr);

            //store the time taken and update the cost.
            path_cost = calculate_cost(goalPtr, x_size, y_size, startPtr, openPtrList, closedPtrList, map, collision_thresh, t);

            //if the path cost is less than the minimum cost, see if it satisfies the time limitation
            tmpPtr = openPtrList[goalKey];
            path_time = tmpPtr->time_steps;
            if (path_time < t){
                if (min_cost){
                    if (min_cost>path_cost);
                        min_cost = path_cost;
                        aimPtr = openPtrList[goalKey];
                }
                else{
                    min_cost = path_cost;
                    aimPtr = openPtrList[goalKey];
                }
            }

        }

        //if at goal pos, wait there.
        key = GETMAPINDEX(aimPtr->posX, aimPtr->posY, x_size, y_size);
        action_x[key] = aimPtr->posX;
        action_y[key] = aimPtr->posY;

        currentPtr = aimPtr;
        previousPtr = aimPtr->parent;

        //retrace the path back  to start node.
        while(previousPtr != NULL){
            key = GETMAPINDEX(previousPtr->posX, previousPtr->posY, x_size, y_size);
            action_x[key] = currentPtr->posX;
            action_y[key] = currentPtr->posY;

            currentPtr = previousPtr;
            previousPtr = currentPtr->parent;
        }
    }

    key = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);

    //set the action
    action_ptr[0] = action_x[key];
    action_ptr[1] = action_y[key];
    
    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}
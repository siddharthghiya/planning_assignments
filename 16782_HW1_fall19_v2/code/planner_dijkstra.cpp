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
#include<unordered_map>
#include <chrono> 
using namespace std::chrono;

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
    // double h;
    double g;
    int cost;
    node* parent;
    int t;
};

struct CompareG{
    bool operator()(const node* lhs, const node* rhs) const
    {
        return (lhs->g) > (rhs->g);
    }
};

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

    // various important instantiations
    priority_queue<node*, vector<node*>, CompareG> open_list;
    vector<node*> closed_list;
    node* currentPtr;
    node* aimPtr;
    node* previousPtr;
    unordered_map<int, node*> openPtrList;
    unordered_map<int, bool> closedPtrList;
    int neighborX, neighborY, neighborKey, currentKey, key, minCost, time;
    static unordered_map<int, int> actions_x;
    static unordered_map<int, int> actions_y;

    //Create the start_node pointer and store it's values, push it into the open_list.
    node* startPtr = new node();
    startPtr->posX = robotposeX; 
    startPtr->posY = robotposeY;
    open_list.push(startPtr);
    openPtrList[GETMAPINDEX(startPtr->posX, startPtr->posY, x_size, y_size)] = startPtr;  
    
    if (curr_time == 0){
        //forward Dijkstra
        while(!open_list.empty()){
            //remove the top node from the open_list with the minimum f value and add it to the closed list.
            currentPtr = open_list.top();
            open_list.pop();
            closed_list.push_back(currentPtr);
            currentKey = GETMAPINDEX(currentPtr->posX, currentPtr->posY, x_size, y_size);

            //if the pointer has already been popped, continue.
            if (closedPtrList[currentKey]){
                continue;   
            }

            closedPtrList[currentKey] = true;

            //go through the neighbours of the current node.
            for (int dir = 0; dir < NUMOFDIRS; dir++){
                int neighborX = currentPtr->posX + dX[dir];
                int neighborY = currentPtr->posY + dY[dir];
                
                //only if the neighbor is within bounds and within collision threshold
                if ((neighborX >= 1 && neighborX <= x_size && neighborY >= 1 && neighborY <= y_size) && ((int)map[GETMAPINDEX(neighborX,neighborY,x_size,y_size)] < collision_thresh)){

                    // generate the neighbor key.
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
                            neighborPtr->parent = currentPtr;
                            neighborPtr->t = currentPtr->t + 1;
                            open_list.push(neighborPtr);
                        }
                    }
                    else{
                        neighborPtr = new node();
                        neighborPtr->posX = neighborX;
                        neighborPtr->posY = neighborY;
                        neighborPtr->cost = (int)map[GETMAPINDEX(neighborX,neighborY,x_size,y_size)];
                        neighborPtr->g = currentPtr->g + neighborPtr->cost;
                        neighborPtr->parent = currentPtr;
                        neighborPtr->t = currentPtr->t + 1;
                        openPtrList[neighborKey] = neighborPtr;
                        open_list.push(neighborPtr);
                    }
                }
            }
        }

        // find the goal with reachable time and minimum cost.
        for (int i =0; i<target_steps; i++){
            currentKey = GETMAPINDEX(target_traj[i], target_traj[i+target_steps], x_size, y_size);
            currentPtr = openPtrList[currentKey];

            if (i > currentPtr->t){
                if (minCost){
                    if (minCost > currentPtr->g + (i-currentPtr->t)*(currentPtr->cost)){
                        minCost = currentPtr->g + (i-currentPtr->t)*(currentPtr->cost);
                        aimPtr = currentPtr;
                    }
                }
                else{
                    minCost = currentPtr->g + (i-currentPtr->t)*(currentPtr->cost);
                    aimPtr = currentPtr;
                }
            }
        }

        //we have the goal position. now set the actions according to it.
        currentKey = GETMAPINDEX(aimPtr->posX, aimPtr->posY, x_size, y_size);
        currentPtr = aimPtr;
        actions_x[currentKey] = currentPtr->posX;
        actions_y[currentKey] = currentPtr->posY;
        previousPtr = currentPtr->parent;

        while(previousPtr){
            key = GETMAPINDEX(previousPtr->posX, previousPtr->posY , x_size, y_size);
            actions_x[key] = currentPtr->posX;
            actions_y[key] = currentPtr->posY;
            currentPtr = previousPtr;
            previousPtr = currentPtr->parent;
        }
    }

    key = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
    
    //set the action
    action_ptr[0] = actions_x[key];
    action_ptr[1] = actions_y[key];

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
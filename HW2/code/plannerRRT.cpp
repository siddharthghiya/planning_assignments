#include "RRTheader.h"

void plannerRRT(
	double*	map,
	int x_size,
	int y_size,
	double* armstart_anglesV_rad,
	double* armgoal_anglesV_rad,
	int numofDOFs,
	double*** plan,
	int* planlength)
{	
	//import initialisations
	node* qStartPtr = new node(numofDOFs);
	node* qGoalPtr = new node(numofDOFs);
	node* qRandPtr;
	node* temp;
	bool goalSelected = false;
	int reached = 0;
	double r;
	int numOfSamples;
	*plan = NULL;
	*planlength = 0;
	int steps = 0;
	bool infinitEpsilon = false;

	//if either of the start or goal configuration is invalid, print that planning is not possible.
	if (!IsValidArmConfiguration(armstart_anglesV_rad, numofDOFs, map, x_size, y_size) || !IsValidArmConfiguration(armgoal_anglesV_rad, numofDOFs, map, x_size, y_size)){
		cout << "Provide a valid start and goal position" << endl;
		return;
	}

	//declare start and goal ptrs
	for (int j=0; j<numofDOFs; j++){
		qStartPtr->angles[j] = armstart_anglesV_rad[j];
		qGoalPtr->angles[j] = armgoal_anglesV_rad[j];
	}

	//form a new pointer to a tree.
	tree* treePtr = new tree(qStartPtr, numofDOFs, map, x_size, y_size);

	while(true){
		//with a probability P_G, choose qGoalPtr as the qRandPtr. With (1-P_G) probability, choose a random point as qRandPtr.
		r = ((double) rand() / (RAND_MAX));
		if (r<P_G){
			qRandPtr = qGoalPtr;
			goalSelected = true;
		}
		else{
			goalSelected = false;
			qRandPtr = sampleNode(numofDOFs);
		}

		//extend the tree towards qRandPtr
		reached = treePtr->extend(qRandPtr, infinitEpsilon);

		//if we reach qRandPtr and qRandPtr == qGoalPtr, break.
		if ((reached==2) && goalSelected){
			break;
		}

		//if number of steps == 1000, the planner is stuck.
		if (steps > 100000){
			cout << "number of samples exceeded 100000, try again" << endl;
			return;
		}
		steps++;
	}

	//set the plan.
	temp = qGoalPtr;
	numOfSamples = qGoalPtr->t;

	if(numOfSamples < 2){
		printf("the arm is already at the goal\n");
		return;
	}
	*plan = (double**) malloc(numOfSamples*sizeof(double*));
	int firstinvalidconf = 1;
	for (int i = numOfSamples-1; i>0; i--){
		(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
		for(int j = 0; j < numofDOFs; j++){
			(*plan)[i][j] = temp->angles[j];
		}
		if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf)
		{
			firstinvalidconf = 1;
			printf("ERROR: Invalid arm configuration!!!\n");
		}
		temp = temp->parent;
	}    
	*planlength = numOfSamples;
	return;
}





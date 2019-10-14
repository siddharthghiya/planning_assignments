#include "RRTheader.h"

void plannerRRTConnect(
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
	node* qNewExpandPtr;
	node* qLastForwardPtr;
	node* qLastBackwardPtr;
	node* temp;
	int reached = 0;
	double r;
	int numOfSamples;
	int numOfSamplesForward;
	int numOfSamplesBackward;
	int reachedExpand;
	int reachedConnect;
	*plan = NULL;
	*planlength = 0;
	int steps = 0;
	tree* treeExtend;
	tree* treeConnect;

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

	//form two new pointers to trees.
	tree* forwardTreePtr = new tree(qStartPtr, numofDOFs, map, x_size, y_size);
	tree* backwardTreePtr = new tree(qGoalPtr, numofDOFs, map, x_size, y_size);

	while(true){
		steps++;

		//if odd, extend the forward tree and connect the backward tree. vice versa if even,
		if (steps/2){
			treeExtend = backwardTreePtr;
			treeConnect = forwardTreePtr;
		}
		else{
			treeExtend = forwardTreePtr;
			treeConnect = backwardTreePtr;	
		}

		//for the expansion tree, choose a random configuration to explore.
		qRandPtr = sampleNode(numofDOFs);

		//extend the expandion tree towards the random pointer.
		reachedExpand = treeExtend->extend(qRandPtr, false);

		//if trapped, continue.
		if (reachedExpand == 1){
			continue;
		}

		//if the expansiontree was expanded either by epsilon or if it reached qRandPtr, try to connect the connectTree to the latest configuration added in the expansionTree.
		qNewExpandPtr = (treeExtend->nodesPtrList).back();
		reachedConnect = treeConnect->extend(qNewExpandPtr, true);

		//if number of steps == 1000, the planner is stuck.
		if (steps > 100000){
			cout << "number of samples exceeded 100000, try again" << endl;
			return;
		}

		//if the connect tree reached qLastAdded, we should have path. Break the loop.
		if (reachedConnect == 2){
			break;
		}
	}

	qLastForwardPtr = (forwardTreePtr->nodesPtrList).back();
	qLastBackwardPtr = (backwardTreePtr->nodesPtrList).back();
	numOfSamplesForward = qLastForwardPtr->t; 
	numOfSamplesBackward = qLastBackwardPtr->t;
	numOfSamples = numOfSamplesForward + numOfSamplesBackward;

	if(numOfSamples < 2){
		printf("the arm is already at the goal\n");
		return;
	}
	*plan = (double**) malloc(numOfSamples*sizeof(double*));

	//set the plan
	temp = qLastForwardPtr;
	for (int i = numOfSamplesForward-1; i>0; i--){
		(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
		for(int j = 0; j < numofDOFs; j++){
			(*plan)[i][j] = temp->angles[j];
		}
		if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size))
		{
			printf("ERROR: Invalid arm configuration!!!\n");
		}
		temp = temp->parent;
	}

	temp = qLastBackwardPtr->parent;
	for (int i = numOfSamplesForward; i<numOfSamples; i++){
		(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
		for(int j = 0; j < numofDOFs; j++){
			(*plan)[i][j] = temp->angles[j];
		}
		if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size))
		{
			printf("ERROR: Invalid arm configuration!!!\n");
		}
		temp = temp->parent;
	}

	*planlength = numOfSamples;
	return;
}

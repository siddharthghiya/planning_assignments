#include "RRTheader.h"

//RRT
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

	//if either of the start or goal configuration is invalid, print that planning is not possible.
	if (!IsValidArmConfiguration(armstart_anglesV_rad, numofDOFs, map, x_size, y_size)){
		cout << "Provide a valid start position" << endl;
		return;
	}

	if (!IsValidArmConfiguration(armgoal_anglesV_rad, numofDOFs, map, x_size, y_size)){
		cout << "Provide a valid goal position" << endl;
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
		reached = treePtr->extend(qRandPtr, false);

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
	for (int i = numOfSamples-1; i>=0; i--){
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

//RRTConnect
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
	node* qNewExtendPtr;
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
	tree* treeExtendPtr;
	tree* treeConnectPtr;

	//if either of the start or goal configuration is invalid, print that planning is not possible.
	if (!IsValidArmConfiguration(armstart_anglesV_rad, numofDOFs, map, x_size, y_size)){
		cout << "Provide a valid start position" << endl;
		return;
	}
	if (!IsValidArmConfiguration(armgoal_anglesV_rad, numofDOFs, map, x_size, y_size)){
		cout << "Provide a valid goal position" << endl;
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
			treeExtendPtr = backwardTreePtr;
			treeConnectPtr = forwardTreePtr;
		}
		else{
			treeExtendPtr = forwardTreePtr;
			treeConnectPtr = backwardTreePtr;	
		}

		//for the extend tree, choose a random configuration to explore.
		qRandPtr = sampleNode(numofDOFs);

		//extend the extend tree towards the random pointer.
		reachedExpand = treeExtendPtr->extend(qRandPtr, false);

		//if trapped, continue.
		if (reachedExpand == 1){
			continue;
		}

		//if the extend tree was expanded either by epsilon or if it reached qRandPtr, try to connect the connectTree to the latest configuration added in the extend Tree.
		qNewExtendPtr = (treeExtendPtr->nodesPtrList).back();
		reachedConnect = treeConnectPtr->extend(qNewExtendPtr, true);

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
	for (int i = numOfSamplesForward-1; i>=0; i--){
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

//RRTStar
void plannerRRTStar(
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

	//if either of the start or goal configuration is invalid, print that planning is not possible.
	if (!IsValidArmConfiguration(armstart_anglesV_rad, numofDOFs, map, x_size, y_size)){
		cout << "Provide a valid start position" << endl;
		return;
	}

	if (!IsValidArmConfiguration(armgoal_anglesV_rad, numofDOFs, map, x_size, y_size)){
		cout << "Provide a valid goal position" << endl;
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
		reached = treePtr->extend(qRandPtr, false);

		//repair the existing tree.
		treePtr->repair();

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
	for (int i = numOfSamples-1; i>=0; i--){
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
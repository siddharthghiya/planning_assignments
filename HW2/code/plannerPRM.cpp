#include "common_header.h"

void plannerPRM(
	double*	map,
	int x_size,
	int y_size,
	double* armstart_anglesV_rad,
	double* armgoal_anglesV_rad,
	int numofDOFs,
	double*** plan,
	int* planlength)
{	
	//important initialisations.
	node* qNewPtr;
	graph* PRMgraph = new graph(numofDOFs, map, x_size, y_size);
	node* qStartPtr = new node(numofDOFs);
	node* qGoalPtr = new node(numofDOFs);
	bool startConnect;
	bool goalConnect;
	bool pathExists;
	node* temp;
	int numOfSamples;
	time_t startTime = time(NULL);
	time_t endTime = time(NULL);

	//constructing a graph.
	for (int i = 0; i<MAX_ITERATIONS_PRM; i++){
		//randomly sample a configuration. sampleNOde will always return a valid configuration.
		qNewPtr = sampleValidNode(numofDOFs, map, x_size, y_size);

		//set current i as identification of qNewPtr.
		qNewPtr->iden = i;

		//add the node to the graph. Connect the new configuration to at most K nearest neighbors. Only connect to neighbors which themselves have less than K neighbors.
		PRMgraph->add_node(qNewPtr);
	}

	//add the start configuration and the goal configuration to the graph.
	//declare start and goal ptrs
	for (int j=0; j<numofDOFs; j++){
		qStartPtr->angles[j] = armstart_anglesV_rad[j];
		qGoalPtr->angles[j] = armgoal_anglesV_rad[j];
	}

	//try to conenct start and goal locally
	startConnect = PRMgraph->local_connect(qStartPtr);
	goalConnect = PRMgraph->local_connect(qGoalPtr);

	// //if start and goal have been connected to the graph, see if a path exists.
	pathExists = PRMgraph->astar_path(qStartPtr, qGoalPtr);
	numOfSamples = qGoalPtr->t + 1;

	if (pathExists){
		temp = qGoalPtr;
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
		cout << "cost of the path is " << qGoalPtr->cost << endl;
		*planlength = numOfSamples;
	}

	else{
		cout << "path not found in 30,000 samples, try again " << endl;
	}

	delete qStartPtr;
	delete qGoalPtr;
	delete PRMgraph;
	endTime = time(NULL);
	cout << "time taken " << int(endTime - startTime) + 1 << endl;
	return;
}

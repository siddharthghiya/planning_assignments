#include "RRTheader.h"

//randomly sample a configuration.
node* sampleValidNode(int numofDOFs, double*	map, int x_size, int y_size){
	node* temp = new node(numofDOFs);
	while(true){
		for(int i=0; i<numofDOFs; i++){
			temp->angles[i] = 2*PI*((double) rand() / (RAND_MAX));
		}
		// check if the configuration is valid or not, if not then sample another point.
		if (IsValidArmConfiguration((temp->angles), numofDOFs, map, x_size, y_size)){
			break;
		}
	}
	return temp;
}

//grpah initialisation
graph::graph(int numofDOFs_, double* map_, int x_size_, int y_size_){
	numofDOFs = numofDOFs_;
	map = map_;
	x_size = x_size_;
	y_size = y_size_;
}

//function for connecting a node straight to the nearest point in graph. Useful for adding start and goal confirgurations to already constructed graph.
bool graph::local_connect(node* qPtr){
	double minDistance = PI/5;
	double distance;
	node* temp;
	node* qNearPtr;
	bool connected = false;

	for (auto i : nodesPtrMap){
		temp = i.second;

		//get the distance of this node from qPtr.
		distance = compute_distance(qPtr, temp);

		if (distance<minDistance){
			//check if the path is collision free.
			if (collision_free(qPtr, temp)){
				qNearPtr = temp;
				connected = true;
			}
		}
	}

	//if connected,
	if (connected){
		(qPtr->neighbors).push_back(qNearPtr);
		(qNearPtr->neighbors).push_back(qPtr);
	}

	return connected;
}

//takes in a new pointer to a node, check it's neighbors and adds it to the graph.
void graph::add_node(node* qNewPtr){
	//add edges. This function will add atmost K edges. Only try to add edges in the graph if there already are nodes in the graph.
	add_edges(qNewPtr);

	array<double, DOF> key;
	//add the node to nodesPtrMap.
	for (int i=0; i<numofDOFs; i++){
		key[i] = qNewPtr->angles[i];
	}

	nodesPtrMap[key] = qNewPtr;
}

//generates nearest neighbors and adds it to nearestNeighbors.
void graph::add_edges(node* qNewPtr){
	node* temp;
	int distance;
	int neighborsAdded = 0;

	//iterate through all the configurations in the unorderedmap to find all configurations within a particular radius of qNewPtr. Add this to nearestNeighbors vector.
	for (auto i : nodesPtrMap){
		//add atmost K neighbors.
		if (neighborsAdded>K){
			break;
		}

		temp = i.second;
		if ((temp->neighbors).size() < K){
			distance = compute_distance(qNewPtr, temp);
			
			// if number of neighbors of nodes nearest to qNewPtr is less than K, add that node as a neighbor of qNewPtr and vice versa. Add at most K neighbors.
			if (distance<NEAR_RADIUS){	
				//only add node to graph if the path between temp and qNewPtr is collision free.
				if (collision_free(qNewPtr, temp)){
					(qNewPtr->neighbors).push_back(temp);
					(temp->neighbors).push_back(qNewPtr);
					neighborsAdded++;
				}
			}
		}
	}
}

double graph::compute_distance(node* q1Ptr, node* q2Ptr){
	double distance = 0;
	double tempDistance;
	for (int i=0; i<numofDOFs; i++){
		tempDistance = (q2Ptr->angles[i] - q1Ptr->angles[i]);
		distance += tempDistance*tempDistance;
	}
	distance = sqrt(distance);

	return distance;
}

int graph::collision_free(node* q1Ptr, node* q2Ptr){
	double direction[numofDOFs];
	double distance;
	int epsilon;
	double angles[numofDOFs];
	int steps = 0;

	//setting the increment direction
	distance = compute_distance(q1Ptr, q2Ptr);
	for (int i=0; i<numofDOFs; i++){
		direction[i] = (q2Ptr->angles[i] - q1Ptr->angles[i])/distance;
	}
	//checking for collision
	//set starting angles to q1Ptr's angles
	for(int i=0; i<numofDOFs; i++){
		angles[i] = q1Ptr->angles[i];
	}
	//loop through epsilon.
	epsilon = (int)(distance/(MAX_STEP_SIZE));
	for (int i=0; i<epsilon; i++){

		//incrementing the angles by tiny amount.
		for (int j=0; j<numofDOFs; j++){
			angles[j] += (MAX_STEP_SIZE)*direction[j];
		}

		//check if the current angles configuration is valid. If invalid, break loop.
		if (!IsValidArmConfiguration(angles, numofDOFs, map, x_size, y_size)){
			break;
		}

		steps++;
	}

	//if steps != epsilon, then collision check failed.
	if (steps != epsilon){
		return 0;
	}

	return 1;
}

//set parents for path between q1Ptr and q2Ptr
bool graph::astar_path(node* q1Ptr, node* q2Ptr){
	priority_queue<node*, vector<node*>, CompareG> openList;
	unordered_map<array<double,5>, node*>  openPtrList;
	unordered_map<array<double,5>, bool>  closedPtrList;
	node* currentPtr;
    node* aimPtr;
    node* previousPtr;
    node* neighborPtr;
    int numNeighbors;
    array<double, DOF> key;
    bool pathFound = false;

    //add q1Ptr to the list.
    openList.push(q1Ptr);

    while(!openList.empty()){
    	//get the pointer to node with lowest g value. 
    	currentPtr = openList.top();
    	openList.pop();

    	//if q2Ptr had been popped, stop.
    	if (currentPtr == q2Ptr){
    		pathFound = true;
    		break;
    	}

    	//add the node to nodesPtrMap.
		for (int i=0; i<numofDOFs; i++){
			key[i] = currentPtr->angles[i];
		}

    	//if the pointer has already been popped, continue.
        if (closedPtrList[key]){
            continue;   
        }

        closedPtrList[key] = true;

        //calculate num of neighbours of currentPtr.
        numNeighbors = (currentPtr->neighbors).size();

        //loop thorugh all the neighbors of currentPtr.
        for (int i = 0; i<numNeighbors; i++){
        	neighborPtr = (currentPtr->neighbors)[i];

        	//find key
			for (int i=0; i<numofDOFs; i++){
				key[i] = neighborPtr->angles[i];
			}

        	//if the neighbor ptr is already in closed list, skip it.
        	if (closedPtrList[key]){
        		continue;
        	}

        	if (openPtrList[key]){
                neighborPtr = openPtrList[key];
                //update the gvalue and the parent
                if(neighborPtr->cost > currentPtr->cost + compute_distance(currentPtr, neighborPtr)){
                    neighborPtr->cost = currentPtr->cost + compute_distance(currentPtr, neighborPtr);
                    neighborPtr->parent = currentPtr;
                    neighborPtr->t = currentPtr->t + 1;
                    openList.push(neighborPtr);
                }
            }
            else{
                neighborPtr->cost = currentPtr->cost + compute_distance(currentPtr, neighborPtr);
                neighborPtr->parent = currentPtr;
                neighborPtr->t = currentPtr->t + 1;
                openPtrList[key] = neighborPtr;
                openList.push(neighborPtr);
            }
        }
    }

    return pathFound;
}
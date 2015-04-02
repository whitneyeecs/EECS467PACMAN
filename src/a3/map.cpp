#include "map.hpp"

Map::Map() { }

vector <Map::Node> Map::initilizeEasy() { 

	vector <Node> map (NUM_NODES_EASY);

	Node n;
	int i = 0;

	//node 0
	n.nodeNum = 0;
	n.right = 0;
	n.left = 0;
	n.front = 0;
	n.behind = 0;

	map[i] = n;
	i++;

	//node 1
	

	return map;
}
 
vector <Map::Node> Map::initilizeMed() { 

	vector <Node> map (NUM_NODES_MED);

	return map;

}

vector <Map::Node> Map::initilizeHard() { 
	
	vector <Node> map (NUM_NODES_HARD);

	return map;
	
}

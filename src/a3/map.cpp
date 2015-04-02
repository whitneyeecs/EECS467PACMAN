#include "map.hpp"

Map::Map() { }

vector <Map::Node> Map::initilizeEasy() { 

	vector <Node> maptmp (NUM_NODES_EASY);
	
	return maptmp;
}
 
vector <Map::Node> Map::initilizeNorm() { 
	
	vector <Node> maptmp (NUM_NODES_NORM);

	return maptmp;
	
}

void Map::getMap(char mapType) {

	if (mapType == 'e')
		map = initilizeEasy();
	else if (mapType == 'n')
		map = initilizeNorm();
	else {
		cout << "Invalid map type." << endl;
		exit(1);
	}
}

#include "map.hpp"

Map::Map() { }

vector <Map::Node> Map::initilize() { 

	vector <Node> maptmp;
	std::ifstream in;
	in.open ("../../data/pacNodes.txt");
	
	int x;

	while(in.good()) {
		Node n;

		in >> x;
		n.nodeNum = x;
		
		in >> x;
		n.xPos = x;

		in >> x;
		n.yPos = x;

		in >> x;
		n.up = x; 

		in >> x;
		n.down = x; 

		in >> x;
		n.left = x; 

		in >> x; 
		n.right = x;  
  
		maptmp.push_back(n);  
	}

	return maptmp;
	
}

void Map::getMap() {

	map = initilize();  

}

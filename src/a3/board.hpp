#ifndef BOARD_HPP
#define BOARD_HPP

#include "math/point.hpp"

#include <lcmtypes/maebot_pose_t.hpp>

#include "Constants.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <stack>

using namespace std;

class Board { 

private:

	vector < vector <char> > route; 
	stack < eecs467::Point<int> > path;

	bool pathFinder(vector < vector <char> >& route, queue < eecs467::Point<int> >& q, 
			eecs467::Point<int>& start, eecs467::Point<int>& end, 
				eecs467::Point<int>& current, char dir);

	void makePath(vector < vector <char> >& route, eecs467::Point<int>& start, 
		eecs467::Point<int>& end, eecs467::Point<int>& current);

	void printRoute(vector < vector <char> >& route);
	
public: 
 
	const static int height = 17;
	const static int width = 19;

	/* represents the pacman board has a 2d vector: (W)aypoints, (P)ath, (X)nonvalid */
	vector < vector <char> > board;
	 
	/* intilizes the board */ 
	Board();
 
	/* prints the Board */
	void printBoard();

	/*converts a maebot_pose_t to board coordinates */
	eecs467::Point<int> convertToBoardCoords(maebot_pose_t pose);

	/*converts board coordinates to global coordinates */
	eecs467::Point <float> convertToGlobalCoords(eecs467::Point<int> p);

	/* pacman function, takes a direction and returns a waypoint location */
	eecs467::Point<int> nextWaypoint(eecs467::Point<int> location, char direction); 
	
	/* returns a list of waypoints in grid coordinates */
	stack < eecs467::Point<int> > getPath(eecs467::Point<int>& start, 
											eecs467::Point<int>& end);

}; 
 
#endif 

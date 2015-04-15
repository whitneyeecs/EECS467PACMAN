#ifndef BOARD_HPP
#define BOARD_HPP

#include "math/point.hpp"
 
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

	bool pathFinder(vector < vector <char> >& route, queue < eecs467::Point<int> >&q, 
			eecs467::Point<int>& start, eecs467::Point<int>& end, 
				eecs467::Point<int>& current, char dir);

	void makePath(vector < vector <char> >& route, eecs467::Point<int>& start, 
		eecs467::Point<int>& end, eecs467::Point<int>& current);

	void printRoute(vector < vector <char> >& route);
	
public: 
 
	const static int height = 17;
	const static int width = 19;

	vector < vector <char> > board;
	 
	/* intilizes the board */ 
	Board();
 
	/* prints the Board */
	void printBoard();

	/* pacman function, takes a direction and returns a waypoint location */
	eecs467::Point<int> nextWaypoint(eecs467::Point<int> location, char direction); 
	
	/* returns a path of directions. 
		u - up, d - down, l - left, r - right */
	stack < eecs467::Point<int> > getPath(eecs467::Point<int>& start, 
											eecs467::Point<int>& end);

}; 
 
#endif 

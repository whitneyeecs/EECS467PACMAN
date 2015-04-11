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
	stack <char> path;

	void clearRoute();

	bool pathFinder(queue < eecs467::Point<int> >& q, eecs467::Point<int>& start, 
				eecs467::Point<int>& end, eecs467::Point<int>& current, char dir);

	void makePath(eecs467::Point<int>& start, eecs467::Point<int>& end, 
				eecs467::Point<int>& current);
	
public: 

	const static int heightInCells = 17;
	const static int widthInCells = 19;

	vector < vector <char> > board;
	
	Board();

	/* initilizes the board */
	void initilize();  
 
	/* prints the Board */
	void printBoard();

	/* prints the steps taken to find a path on the board */
	void printRoute();
	
	/* returns a path of directions. 
		u - up, d - down, l - left, r - right */
	stack <char> getPath(eecs467::Point<int>& start, eecs467::Point<int>& end);

}; 
 
#endif 

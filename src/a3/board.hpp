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

	vector < vector <char> > makeRoute();

	/* prints the steps taken to find a path on the board */
	void printRoute(vector < vector <char> >& route);
	
public: 
 
	const static int heightInCells = 17;
	const static int widthInCells = 19;

	vector < vector <char> > board;
	 
	/* intilizes the board */ 
	Board();
 
	/* prints the Board */
	void printBoard();
	
	/* returns a path of directions. 
		u - up, d - down, l - left, r - right */
	stack < eecs467::Point<int> > getPath(eecs467::Point<int>& start, 
											eecs467::Point<int>& end);

}; 
 
#endif 

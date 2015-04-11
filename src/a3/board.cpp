#include "board.hpp"

Board::Board() { }

void Board::initilize() { 
	
	vector < vector <char> > tmp;

	std::fstream in;

	in.open("../data/boardGrid.txt", std::ifstream::in);

	//initilize Board
	if(in.good()) {

		char c;

		for(int i = 0; i < widthInCells; i++) {
			vector <char> row;
			for(int j = 0; j < heightInCells; j++) {
				in >> c;
				row.push_back(c);
			}
			tmp.push_back(row);
		}
	}

	board = tmp;
	
	in.close();	 

	tmp.clear();

	//initilize board for placing the route
	for(int i = 0; i < widthInCells; i++) {
			vector <char> row;
		for(int j = 0; j < heightInCells; j++) {
			row.push_back(board[i][j]);
		}
		tmp.push_back(row);
	}

	route = tmp;
}

void Board::printBoard() {

	for(int i = 0; i < widthInCells; i++) {
		for(int j = 0; j < heightInCells; j++) {
			cout << board[i][j] << " ";
		}
		cout << endl;
	}
	cout << endl << endl;
}

void Board::printRoute() {

	for(int i = 0; i < widthInCells; i++) {
		for(int j = 0; j < heightInCells; j++) {
			cout << route[i][j] << " ";
		}
		cout << endl;
	}

	cout << endl << endl;
}

void Board::clearRoute() {

	vector <vector <char> > tmp;

	for(int i = 0; i < widthInCells; i++) {
		for(int j = 0; j < heightInCells; j++) 
			route[i][j] = board[i][j];
	}
	
	route = tmp;
}

stack <char> Board::getPath(eecs467::Point<int>& start, eecs467::Point<int>& end) {

		clearRoute();
		
		while(!path.empty())
			path.pop();

		queue < eecs467::Point<int> > q;

		//initilize and queue start location
		eecs467::Point<int> current;
		current.x = start.x;
		current.y = start.y;

		eecs467::Point<int> tmpS;
		tmpS.x = start.x;
		tmpS.y = start.y;		
		q.push(tmpS);

		route[start.x][start.y] = 'q';

		while(!q.empty()) {

			//remove next location
			current.x = q.front().x;
			current.y = q.front().y;
			q.pop();

			//check adjacent locations, no bias towards waypoints

			current.x--; //up
			if(pathFinder(q, start, end, current, 'u'))
				return path;
				
			current.x++;
			current.y++; //right
			if(pathFinder(q, start, end, current, 'r'))
				return path;

			current.y--;
			current.x++; //down
			if(pathFinder(q, start, end, current, 'd'))
				return path;

			current.x--;
			current.y--; //left
			if(pathFinder(q, start, end, current, 'l'))
				return path;
		}

		return path;
}

bool Board::pathFinder(queue < eecs467::Point<int> >& q, eecs467::Point<int>& start, 
			eecs467::Point<int>& end, eecs467::Point<int>& current, char dir) {

	//out of bounds
	if( current.x >= widthInCells || current.x < 0 || current.y >= heightInCells 
		|| current.y < 0) 
		return false;
	
	//invalid move
	if( route[current.x][current.y] == 'X' || route[current.x][current.y] == 'q')
		return false;

	//already been added to the queue
	if( route[current.x][current.y] == 'u' || route[current.x][current.y] == 'r' ||
		route[current.x][current.y] == 'd' || route[current.x][current.y] == 'l')
		return false;
	
	//note direction of movement
	route[current.x][current.y] = dir;

	//reached the end point
	if( (current.x == end.x) && (current.y == end.y) ) {
		makePath(start, end, current);
		return true;
	}
		
	//otherwise
	eecs467::Point<int> que; 
	que.x = current.x;
	que.y = current.y;
	q.push(que);

	return false;
}

void Board::makePath(eecs467::Point<int>& start, eecs467::Point<int>& end, 
						eecs467::Point<int>& current) {

	current.x = end.x;
	current.y = end.y;
	
	while( (current.x != start.x) && (current.y != start.y) ) {

		if(route[current.x][current.y] == 'u') {
			current.x++;
			route[current.x][current.y] = 'u';
			char dir = 'u';
			path.push(dir);
		}
	
		else if(route[current.x][current.y] == 'r') {
			current.y--;
			route[current.x][current.y] = 'r';
			char dir = 'r';
			path.push(dir);
		}

		else if(route[current.x][current.y] == 'd') {
			current.x--;
			route[current.x][current.y] = 'd';
			char dir = 'd';
			path.push(dir);
		}

		else if(route[current.x][current.y] == 'l') {
			current.y++;
			route[current.x][current.y] = 'l';
			char dir = 'l';
			path.push(dir);
		}

		else {
			cout << "Illegal direction parameter" << endl;
			exit(1);
		}
	}
}	


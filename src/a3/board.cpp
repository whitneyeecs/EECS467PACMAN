#include "board.hpp"

Board::Board() { 

	std::fstream in;

	in.open("data/boardGrid.txt", std::ifstream::in);

	vector < vector <char> > tmp;

	//initilize Board
	if(in.good()) {

		board.resize(widthInCells); 
		for(int x =0; x < widthInCells; ++x){
			board[x].resize(heightInCells);
		}

		for(int y = heightInCells -1; y > -1; --y) {
			for(int x = 0; x < widthInCells; ++x) 
				in >> board[x][y];
		}
	}
	
	in.close();	

	

}

void Board::printBoard() {

	for(int y = heightInCells - 1; y > -1; --y) {
		for(int x = 0; x < widthInCells; x++) {
			cout << board[x][y] << " ";
		}
		cout << endl;
	}
	cout << endl << endl;
}

vector < vector <char> > Board::makeRoute() {
}

stack <char> Board::getPath(eecs467::Point<int>& start, eecs467::Point<int>& end) {
		
		//clear the last path
		while(!path.empty())
			path.pop();
	
		//build a temporary board
		vector < vector <char> > route = board;

		//start queue
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

			current.y++; //up
			if(pathFinder(route, q, start, end, current, 'u'))
				return path;
				
			current.y--;
 
			current.x++; //right
			if(pathFinder(route, q, start, end, current, 'r'))
				return path;

			current.x--; 

			current.y--; //down
			if(pathFinder(route, q, start, end, current, 'd'))
				return path;

			current.y++;

			current.x--; //left
			if(pathFinder(route, q, start, end, current, 'l'))
				return path;

			//current.x++;
		}

		return path;
}

bool Board::pathFinder(vector < vector <char> >& route, queue < eecs467::Point<int> >&q, 
			eecs467::Point<int>& start, eecs467::Point<int>& end, 
				eecs467::Point<int>& current, char dir) {

	//out of bounds
	if( current.x >= widthInCells || current.x < 0 || current.y >= heightInCells 
		|| current.y < 0) 
		return false;
	
	//invalid move
	else if( route[current.x][current.y] == 'X' || route[current.x][current.y] == 'q')
		return false;

	//already been added to the queue
	else if( route[current.x][current.y] == 'u' || route[current.x][current.y] == 'r' ||
		route[current.x][current.y] == 'd' || route[current.x][current.y] == 'l')
		return false;
	
	else {
		//note direction of movement
		route[current.x][current.y] = dir;

		//reached the end point
		if( (current.x == end.x) && (current.y == end.y) ) {
			makePath(route, start, end, current);
			cout << "found end" << endl;
			return true;
		}
		
		//otherwise
		cout << "coords " << current.x << " " << current.y << endl;
		eecs467::Point<int> que; 
		que.x = current.x;
		que.y = current.y;
		q.push(que);
		return false;
	}
}

void Board::makePath(vector < vector <char> >& route, eecs467::Point<int>& start, 
		eecs467::Point<int>& end, eecs467::Point<int>& current) {

	//debugging purposes
	printRoute(route);

	//Point<int> next;
	//next.x = current.x;
	//next.y = current.y;

	current.x = end.x;
	current.y = end.y;

	while( !(current.x == start.x && current.y == start.y) ) {

		cout << "coords " << current.x << " " << current.y << " " << 
		route[current.x][current.y] << endl;

		if(route[current.x][current.y] == 'u') {
			current.y--;
			char dir = 'u';
			path.push(dir);
		}
	
		else if(route[current.x][current.y] == 'r') {
			current.x--;
			char dir = 'r';
			path.push(dir);
		}

		else if(route[current.x][current.y] == 'd') {
			current.y++;
			char dir = 'd';
			path.push(dir);
		}

		else if(route[current.x][current.y] == 'l') {
			current.x++;
			char dir = 'l';
			path.push(dir);
		}
 
		else {
			cout << "Illegal direction parameter" << endl;
			exit(1);
		}
	}
}	

void Board::printRoute(vector < vector <char> >& route) {
	
	cout << endl; 

	for(int y = heightInCells - 1; y > -1; --y) {
		for(int x = 0; x < widthInCells; x++) {
			cout << route[x][y] << " ";
		}
		cout << endl;
	}
	cout << endl << endl;
}

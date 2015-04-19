#include "board.hpp"

Board::Board() { 

	std::fstream in;

	in.open("data/boardGrid.txt", std::ifstream::in);

	vector < vector <char> > tmp;

	//initilize Board
	if(in.good()) {

		board.resize(width); 
		for(int x =0; x < width; ++x){
			board[x].resize(height);
		}

		for(int y = height -1; y > -1; --y) {
			for(int x = 0; x < width; ++x) 
				in >> board[x][y];
		}
	}
	
	in.close();	
}

void Board::printBoard() {

	for(int y = height - 1; y > -1; --y) {
		for(int x = 0; x < width; x++) {
			cout << board[x][y] << " ";
		}
		cout << endl;
	}
	cout << endl << endl;
}


eecs467::Point<int> Board::convertToBoardCoords(maebot_pose_t pose) {

	eecs467::Point <int> point;
	point.x = (pose.x + eecs467::boardCellWidth / 2.0)/eecs467::boardCellWidth;
	point.y = (pose.y + eecs467::boardCellHeight / 2.0)/eecs467::boardCellHeight;
	return point;
}

eecs467::Point <float> Board::convertToGlobalCoords(eecs467::Point<int> p) {

	eecs467::Point <float> point;
	point.x = p.x*eecs467::boardCellWidth;
	point.y = p.y*eecs467::boardCellHeight;
	return point;
}

eecs467::Point<int> Board::nextWaypoint(eecs467::Point<int> location, char direction){

	eecs467::Point<int> point;
	point.x = -1;
	point.y = -1;

	cout << point.x << ' ' << point.y << endl;

	if (direction == 'u') {
		for(int i = location.y+1; i < height; i++) {
			if (board[location.x][i] == 'W') {
				point.x = location.x;
				point.y = i;
				return point;
			}
			else if (board[location.x][i] == 'X')
				return point;
		}
	}

	if (direction == 'r') {
		for(int i = location.x+1; i < width; i++) {
			if (board[i][location.y] == 'W') {
				point.x = i;
				point.y = location.y;
				return point;
			}
			else if (board[i][location.y] == 'X') 
				return point; 
		}
	}

	if (direction == 'd') {
		for(int i = location.y-1; i >= 0; i--) {
			if (board[location.x][i] == 'W') {
				point.x = location.x;
				point.y = i;
				return point;
			}
			else if (board[location.x][i] == 'X') 
				return point;
		}
	}

	if (direction == 'l') {
		for(int i = location.x-1; i >= 0; i--) {
			if (board[i][location.y] == 'W') {
				point.x = i;
				point.y = location.y;
				return point;
			}
			else if (board[i][location.y] == 'X') 
				return point;
		}	
	}

	return point;
} 

stack < eecs467::Point<int> > Board::getPath(eecs467::Point<int>& start, 
												eecs467::Point<int>& end) {
		
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
	if( current.x >= width || current.x < 0 || current.y >= height 
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
			cout << "Found End" << endl;
			return true;
		}
		
		//otherwise
		//cout << "coords " << current.x << " " << current.y << endl;
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
	//printRoute(route);

	current.x = end.x;
	current.y = end.y;

	while( !(current.x == start.x && current.y == start.y) ) {

		//cout << "coords " << current.x << " " << current.y << " " << 
		//route[current.x][current.y] << endl;

		if(route[current.x][current.y] == 'u') {
			if(board[current.x][current.y] == 'W') {
				eecs467::Point<int> point;
				point.x = current.x;
				point.y = current.y;
				path.push(point);
			}
			current.y--;
		}
	
		else if(route[current.x][current.y] == 'r') {
			if(board[current.x][current.y] == 'W') {
				eecs467::Point<int> point;
				point.x = current.x;
				point.y = current.y;
				path.push(point);
			}
			current.x--;
		}

		else if(route[current.x][current.y] == 'd') {
			if(board[current.x][current.y] == 'W') {
				eecs467::Point<int> point;
				point.x = current.x;
				point.y = current.y;
				path.push(point);
			}
			current.y++;
		}

		else if(route[current.x][current.y] == 'l') {
			if(board[current.x][current.y] == 'W') { 
				eecs467::Point<int> point;
				point.x = current.x;
				point.y = current.y;
				path.push(point);
			}
			current.x++;
		}
 
		else {
			cout << "Illegal direction parameter" << endl;
			exit(1);
		}
	}
}	

void Board::printRoute(vector < vector <char> >& route) {
	
	cout << endl; 

	for(int y = height - 1; y > -1; --y) {
		for(int x = 0; x < width; x++) {
			cout << route[x][y] << " ";
		}
		cout << endl;
	}
	cout << endl << endl;
}

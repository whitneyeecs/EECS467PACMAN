#include "a3/board.hpp"

#include "math/point.hpp"

using namespace std;

int main() {

	Board board;
	
	board.initilize();
	
	board.printBoard();

	eecs467::Point<int> a;
	eecs467::Point<int> b;

	a.x = 0;
	a.y = 0;

	b.x = 0;
	b.y = 5;
	
	stack <char> path = board.getPath(a, b);

	while(!path.empty()){
		cout << "hello" << endl;
		char c = path.top();
		cout << c << ' ';
		path.pop();
	}

	board.printRoute();

	return 0;

}

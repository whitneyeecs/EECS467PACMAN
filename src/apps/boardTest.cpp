#include "a3/board.hpp"

#include "math/point.hpp"

using namespace std;

int main() {

	Board board; 

	board.printBoard();

	eecs467::Point<int> a;
	eecs467::Point<int> b;

	cout << "Enter start location x,y" << endl;
	cin >>	a.x;
	cin >> a.y;

	cout << "Enter end location x,y" << endl;
	cin >>	b.x;
	cin >> b.y;
	
	stack < eecs467::Point<int> > path = board.getPath(a, b);

	while(!path.empty()){
		eecs467::Point<int> p = path.top();
		cout << p.x << ' ' << p.y << endl;
		path.pop();
	}

	return 0;

} 

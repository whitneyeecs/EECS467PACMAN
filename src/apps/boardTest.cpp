#include "a3/board.hpp"

#include "math/point.hpp"

using namespace std;

int main() {

	Board board;

	board.printBoard();

	eecs467::Point<int> a;
	eecs467::Point<int> b;

	cout << " Enter star location x,y" << endl;
	cin >>	a.x;
	cin >> a.y;

	cout << " Enter end location x,y" << endl;
	cin >>	b.x;
	cin >> b.y;
	
	stack <char> path = board.getPath(a, b);

	while(!path.empty()){
		char c = path.top();
		cout << c << ' ';
		path.pop();
	}
	
	cout << endl;

	return 0;

} 

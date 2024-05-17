// main.cpp : Defines the entry point for the application.
//

#include "main.h"
#include "mesh.h"

using namespace std;

int main()
{
	const unsigned int cMaxPathNodes = 256;
// make a float array with [0,0]
	auto mesh = new Mesh();

	// Output the mesh to console
	cout << mesh->ToString() << endl;

	// Create a start position
	//float startPos[3] = { 15, 0, 35 };
	const float startPos[3] = { 15, 0, 15 };
	//const float startPos[3] = { 0, 0, 0 };

	// Create an end position
	//float endPos[3] = { 25, 0, 5 };
	const float endPos[3] = { 45, 1, 25 };

	// Create a path
	float path[cMaxPathNodes * 3];

	unsigned int pathLength = 0;
	
	// Find the path
	mesh->FindPath(startPos, endPos, path, pathLength, cMaxPathNodes);

	cout << "Path length: " << pathLength << endl;
	// Output the path to console
	for (unsigned int i = 0; i < pathLength; i++)
	{
		cout << "Path[" << i << "]: " << path[i * 3] << ", " << path[i * 3 + 1] << ", " << path[i * 3 + 2] << endl;
	}

	return 0;
}

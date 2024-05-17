#pragma once

#include <string>
#include <array>

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "Recast.h"

// A class called mesh that wraps rcPolyMesh and rcPolyMeshDetail
// It is used to store the data of the mesh and to provide some utility functions
class Mesh
{
	// Constructor
public:
	Mesh();
	~Mesh();

	bool FindPath(const float* startPos, const float* endPos, float* path, unsigned int& pathLength, const unsigned int& cMaxStraightPathNodes);

	//Create ToString function
	std::string ToString();

	// Functions

private:

	bool CreateMesh();
	bool RasterizeRectangle(rcContext &ctx, rcHeightfield &heightField, const std::array<float, 2> &bottomLeft, const std::array<float, 2>& topRight, const float &height = 0);

	// Variables
	dtNavMesh* navMesh_;
	dtNavMeshQuery* navQuery_;

};
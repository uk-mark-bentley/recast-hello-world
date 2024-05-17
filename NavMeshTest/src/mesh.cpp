#include "mesh.h"

#include "DetourNavMeshBuilder.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "Recast.h"
#include "RecastAlloc.h"

#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <string.h>


using namespace std;

Mesh::Mesh() :
	navMesh_(dtAllocNavMesh()),
	navQuery_(dtAllocNavMeshQuery())
{

	CreateMesh();

	// Build a polymesh with a single square polygon centered at the origin and 20m x 20m in size.
	/*polyMesh_->nverts = 4;
	polyMesh_->nvp = 4;
	polyMesh_->npolys = 1;
	polyMesh_->maxpolys = 1;
	polyMesh_->verts = (unsigned short*)rcAlloc(sizeof(unsigned short) * polyMesh_->nverts, RC_ALLOC_PERM);
	polyMesh_->polys = (unsigned short*)rcAlloc(sizeof(unsigned short) * polyMesh_->maxpolys * 2 * polyMesh_->nvp, RC_ALLOC_PERM);
	polyMesh_->regs = (unsigned short*)rcAlloc(sizeof(unsigned short) * polyMesh_->maxpolys, RC_ALLOC_PERM);
	polyMesh_->flags = (unsigned short*)rcAlloc(sizeof(unsigned short) * polyMesh_->maxpolys, RC_ALLOC_PERM);
	polyMesh_->areas = (unsigned char*)rcAlloc(sizeof(unsigned char) * polyMesh_->maxpolys, RC_ALLOC_PERM);*/



}

Mesh::~Mesh()
{
	dtFreeNavMesh(navMesh_);
	dtFreeNavMeshQuery(navQuery_);

}

bool Mesh::FindPath(const float* startPos, const float* endPos, float* path, unsigned int& pathLength, const unsigned int& cMaxStraightPathNodes)
{
	const float cMaxSearchDistance[3] = { 2, 4, 2 };

	dtPolyRef startRef = 0;
	dtPolyRef endRef = 0;

	dtQueryFilter filter;

	//filter.setIncludeFlags(63);
	//Write the filter to console

	cout << "Filter: " << endl;
	cout << "Include flags: " << filter.getIncludeFlags() << endl;
	cout << "Exclude flags: " << filter.getExcludeFlags() << endl;
	cout << "Area cost: " << filter.getAreaCost(0) << endl;
	cout << "Area cost: " << filter.getAreaCost(1) << endl;
	cout << "Area cost: " << filter.getAreaCost(2) << endl;
	cout << "Area cost: " << filter.getAreaCost(3) << endl;
	cout << "Area cost: " << filter.getAreaCost(4) << endl;
	cout << "Area cost: " << filter.getAreaCost(5) << endl;
	cout << "Area cost: " << filter.getAreaCost(6) << endl;


	//filter.setExcludeFlags(0);
	float startPosOnMesh[3];
	float endPosOnMesh[3];
	//Hack while findNearestPoly is not working
	//float startPosOnMesh[3]{ startPos[0], startPos[1], startPos[2] };
	//float endPosOnMesh[3]{ endPos[0], endPos[1], endPos[2] };

	dtStatus status = navQuery_->findNearestPoly(startPos, cMaxSearchDistance, &filter, &startRef, startPosOnMesh);

	if (dtStatusSucceed(status) && startRef > 0)
	{
		cout << "Found start poly" << endl;
	}
	else
	{
		cout << "Failed to find start poly" << endl;
	}

	if (dtStatusInProgress(status))
	{
		cout << "Start poly In progress" << endl;
	}
	else if (dtStatusFailed(status))
	{
		cout << "Failed to find start poly" << endl;
		return false;
	}

	if (dtStatusDetail(status, DT_PARTIAL_RESULT))
	{
		cout << "Partial result" << endl;
	}
	if (dtStatusDetail(status, DT_BUFFER_TOO_SMALL))
	{
		cout << "Buffer too small" << endl;
	}
	if (dtStatusDetail(status, DT_OUT_OF_MEMORY))
	{
		cout << "Out of memory" << endl;
	}
	if (dtStatusDetail(status, DT_WRONG_MAGIC))
	{
		cout << "Wrong magic" << endl;
	}
	if (dtStatusDetail(status, DT_WRONG_VERSION))
	{
		cout << "Wrong version" << endl;
	}
	if (dtStatusDetail(status, DT_INVALID_PARAM))
	{
		cout << "Invalid param" << endl;
	}
	if (dtStatusDetail(status, DT_OUT_OF_NODES))
	{
		cout << "Out of nodes" << endl;
	}
	if (dtStatusDetail(status, DT_ALREADY_OCCUPIED))
	{
		cout << "Already occupied" << endl;
	}

	// Write to console the start position
	cout << "StartRef: " << startRef << endl;
	cout << "Start position: " << startPos[0] << ", " << startPos[1] << ", " << startPos[2] << endl;

	// Write to console the start position on the mesh
	cout << "Start position on mesh: " << startPosOnMesh[0] << ", " << startPosOnMesh[1] << ", " << startPosOnMesh[2] << endl;

	status = navQuery_->findNearestPoly(endPos, cMaxSearchDistance, &filter, &endRef, endPosOnMesh);

	if (dtStatusSucceed(status) && endRef > 0)
	{
		cout << "Found end poly" << endl;
	}
	else
	{
		cout << "Failed to find end poly" << endl;
	}

	cout << "End position: " << endPos[0] << ", " << endPos[1] << ", " << endPos[2] << endl;

	// Write to console the end position on the mesh
	cout << "End position on mesh: " << endPosOnMesh[0] << ", " << endPosOnMesh[1] << ", " << endPosOnMesh[2] << endl;

	const int cMaxPathPolys = 256;
	dtPolyRef pathPolys[cMaxPathPolys];
	int pathCount;

	// Hack while findNearestPoly is not working
	//startRef = navMesh_->encodePolyId(static_cast<const dtNavMesh*>(navMesh_)->getTile(0)->salt, 0, 0);
	//endRef = navMesh_->encodePolyId(static_cast<const dtNavMesh*>(navMesh_)->getTile(0)->salt, 0, 3);

	status = navQuery_->findPath(startRef, endRef, startPosOnMesh, endPosOnMesh, &filter, pathPolys, &pathCount, cMaxPathPolys);

	if (dtStatusFailed(status))
	{
		cout << "Failed to find path" << endl;
		if (dtStatusDetail(status, DT_PARTIAL_RESULT))
		{
			cout << "Partial result" << endl;
		}
		if (dtStatusDetail(status, DT_BUFFER_TOO_SMALL))
		{
			cout << "Buffer too small" << endl;
		}
		if (dtStatusDetail(status, DT_OUT_OF_MEMORY))
		{
			cout << "Out of memory" << endl;
		}
		if (dtStatusDetail(status, DT_WRONG_MAGIC))
		{
			cout << "Wrong magic" << endl;
		}
		if (dtStatusDetail(status, DT_WRONG_VERSION))
		{
			cout << "Wrong version" << endl;
		}
		if (dtStatusDetail(status, DT_INVALID_PARAM))
		{
			cout << "Invalid param" << endl;
		}
		if (dtStatusDetail(status, DT_OUT_OF_NODES))
		{
			cout << "Out of nodes" << endl;
		}
		if (dtStatusDetail(status, DT_ALREADY_OCCUPIED))
		{
			cout << "Already occupied" << endl;
		}
		return false;
	}

	unsigned char straightPathFlags[cMaxStraightPathNodes];
	dtPolyRef straightPathPolys[cMaxStraightPathNodes];
	int straightPathCount;
	int options = 0; // see: dtStraightPathOptions

	status = navQuery_->findStraightPath(startPosOnMesh, endPosOnMesh, pathPolys, pathCount, path, straightPathFlags, straightPathPolys,
		&straightPathCount, cMaxStraightPathNodes, options);

	if (dtStatusFailed(status))
	{
		cout << "Failed to find straight path" << endl;
		if (dtStatusDetail(status, DT_PARTIAL_RESULT))
		{
			cout << "Partial result" << endl;
		}
		if (dtStatusDetail(status, DT_BUFFER_TOO_SMALL))
		{
			cout << "Buffer too small" << endl;
		}
		if (dtStatusDetail(status, DT_OUT_OF_MEMORY))
		{
			cout << "Out of memory" << endl;
		}
		if (dtStatusDetail(status, DT_WRONG_MAGIC))
		{
			cout << "Wrong magic" << endl;
		}
		if (dtStatusDetail(status, DT_WRONG_VERSION))
		{
			cout << "Wrong version" << endl;
		}
		if (dtStatusDetail(status, DT_INVALID_PARAM))
		{
			cout << "Invalid param" << endl;
		}
		if (dtStatusDetail(status, DT_OUT_OF_NODES))
		{
			cout << "Out of nodes" << endl;
		}
		if (dtStatusDetail(status, DT_ALREADY_OCCUPIED))
		{
			cout << "Already occupied" << endl;
		}
		return false;
	}

	for (int i = 0; i < straightPathCount; i++)
	{
		float* node = path + (i * 3);
		cout << "Straight path " << i << ": " << node[0] << ", " << node[1] << ", " << node[2] << endl;
	}

	// Copy the path to the path array
	//memcpy(path, straightPath, straightPathCount * sizeof(float) * 3);

	// Set the path length
	pathLength = straightPathCount;

	return true;
}

std::string Mesh::ToString()
{
	std::ostringstream oss;
	oss << "Mesh Details: " << std::endl;


	return oss.str();
}


bool Mesh::CreateMesh()
{
	rcContext ctx = false;

	float* origin = new float[2];

	float* mesh_top_right = new float[2] {100, 100};
	float* mesh_bottom_left = new float[2] {0, 0};

	// print the values of the arrays
	cout << "origin: " << origin[0] << ", " << origin[1] << endl;
	cout << "mesh_top_right: " << mesh_top_right[0] << ", " << mesh_top_right[1] << endl;
	cout << "mesh_bottom_left: " << mesh_bottom_left[0] << ", " << mesh_bottom_left[1] << endl;

	const float cMinHeight = 0;
	const float cMaxHeight = 100;
	const float cCellSize = 2.0;
	const float cCellHeight = 1.0;

	float* minBounds = new float[3] {mesh_bottom_left[0], cMinHeight, mesh_bottom_left[1]};
	float* maxBounds = new float[3] {mesh_top_right[0], cMaxHeight, mesh_top_right[1]};

	int sizeX = (maxBounds[0] - minBounds[0]) / cCellSize;
	int sizeZ = (maxBounds[2] - minBounds[2]) / cCellSize;

	rcHeightfield* heightfield = rcAllocHeightfield();

	if (!rcCreateHeightfield(&ctx, *heightfield, sizeX, sizeZ, minBounds, maxBounds, cCellSize, cCellHeight))
	{
		rcFreeHeightField(heightfield);
		cout << "Failed to create heightfield" << endl;
		return false;
	}

	cout << "Height: " << heightfield->height << endl;
	cout << "Bmin: " << heightfield->bmin[0] << ", " << heightfield->bmin[1] << ", " << heightfield->bmin[2] << endl;
	cout << "Bmax: " << heightfield->bmax[0] << ", " << heightfield->bmax[1] << ", " << heightfield->bmax[2] << endl;
	cout << "Cell size: " << heightfield->cs << endl;
	cout << "Cell height: " << heightfield->ch << endl;

	//Add a 20x20x20m cube obstace in the middle of the mesh by rasterizing triangles into the heightfield.

	if (!RasterizeRectangle(ctx, *heightfield, std::array<float, 2> {10, 10}, std::array<float, 2> {30, 30}, 0))
	{
		cout << "Failed to rasterize rectangle" << endl;
		rcFreeHeightField(heightfield);
		return false;
	}

	if (!RasterizeRectangle(ctx, *heightfield, std::array<float, 2> {30, 20}, std::array<float, 2> {50, 40}, 0))
	{
		cout << "Failed to rasterize rectangle" << endl;
		rcFreeHeightField(heightfield);
		return false;
	}

	// Write to console the heightfield details including vertices and triangles
	cout << "Heightfield details: " << endl;
	cout << "Width: " << heightfield->width << endl;
	cout << "Height: " << heightfield->height << endl;
	// Write each span to the console
	cout << "Spans: " << endl;
	/*for (int i = 1; i <= heightfield->width * heightfield->height; i++)
	{
		cout << "Span " << i << ": " << endl;
		cout << "Lowest: " << heightfield->spans[i]->smin << endl;
		cout << "Highest: " << heightfield->spans[i]->smax << endl;
	}*/
	for (int x = 0; x < heightfield->width; ++x)
	{
		for (int z = 0; z < heightfield->height; ++z)
		{
			for (rcSpan* s = heightfield->spans[x + z * heightfield->width]; s; s = s->next)
			{
				std::cout << "Span at (" << x << ", " << z << ") area: " << s->area << std::endl;
			}
		}
	}

	rcCompactHeightfield* compactHeightfield = rcAllocCompactHeightfield();

	const int cMinimumCeilingHeightInVox = 0;
	const int cMinClimableLedgeHeightInVox = 10;

	if (!rcBuildCompactHeightfield(&ctx, cMinimumCeilingHeightInVox, cMinClimableLedgeHeightInVox, *heightfield, *compactHeightfield))
	{
		cout << "Failed to build compact heightfield" << endl;
		rcFreeHeightField(heightfield);
		rcFreeCompactHeightfield(compactHeightfield);
		return false;
	}

	rcFreeHeightField(heightfield);



	if (!rcBuildDistanceField(&ctx, *compactHeightfield))
	{
		cout << "Failed to build distance field" << endl;
		rcFreeCompactHeightfield(compactHeightfield);
		return false;
	}

	if (!rcBuildRegions(&ctx, *compactHeightfield, 0, 0, 0))
	{
		cout << "Failed to build regions" << endl;
		rcFreeCompactHeightfield(compactHeightfield);
		return false;
	}

	for (int s = 0; s < compactHeightfield->spanCount; ++s)
	{
		rcCompactSpan& span = compactHeightfield->spans[s];

		std::cout << "Span:" << s << " area: " << static_cast<int>(compactHeightfield->areas[s]) << std::endl;

	}

	//Write to console the compact heightfield details
	cout << "Compact heightfield details: " << endl;
	cout << "Width: " << compactHeightfield->width << endl;
	cout << "Height: " << compactHeightfield->height << endl;
	cout << "Span count: " << compactHeightfield->spanCount << endl;
	cout << "Walkable height: " << compactHeightfield->walkableHeight << endl;
	cout << "Walkable climb: " << compactHeightfield->walkableClimb << endl;
	cout << "Border size: " << compactHeightfield->borderSize << endl;
	cout << "Max distance: " << compactHeightfield->maxDistance << endl;
	cout << "Max regions: " << compactHeightfield->maxRegions << endl;
	cout << "Min x: " << compactHeightfield->bmin[0] << endl;
	cout << "Min y: " << compactHeightfield->bmin[1] << endl;
	cout << "Min z: " << compactHeightfield->bmin[2] << endl;
	cout << "Max x: " << compactHeightfield->bmax[0] << endl;
	cout << "Max y: " << compactHeightfield->bmax[1] << endl;
	cout << "Max z: " << compactHeightfield->bmax[2] << endl;
	cout << "Cell size: " << compactHeightfield->cs << endl;
	cout << "Cell height: " << compactHeightfield->ch << endl;
	// Write the grid to console with a X for no-span and a O for span
	cout << "Grid: " << endl;
	for (int i = 0; i < compactHeightfield->width * compactHeightfield->height; i++)
	{
		if (compactHeightfield->cells[i].count == 0)
		{
			cout << "-";
		}
		else
		{
			cout << "X";
		}
		if ((i + 1) % compactHeightfield->width == 0)
		{
			cout << endl;
		}

	}

	rcContourSet* contourSet = rcAllocContourSet();

	const float cMaxErrorInMetres = 1.0;
	const int cMaxEdgeLengthDisabled = 0;

	if (!rcBuildContours(&ctx, *compactHeightfield, cMaxErrorInMetres, cMaxEdgeLengthDisabled, *contourSet))
	{
		cout << "Failed to build contours" << endl;
		rcFreeCompactHeightfield(compactHeightfield);
		rcFreeContourSet(contourSet);
		return false;
	}



	// Write to console the contour set details
	cout << "Contour set details: " << endl;
	cout << " - Cell size (m):" << contourSet->cs << endl;
	cout << " - Bounding box min (m): " << contourSet->bmin[0] << ", " << contourSet->bmin[1] << ", " << contourSet->bmin[2] << endl;
	cout << " - Number of contours: " << contourSet->nconts << endl;
	float originX = contourSet->bmin[0];
	float originZ = contourSet->bmin[2];
	float cellSize = contourSet->cs;
	for (int i = 0; i < contourSet->nconts; ++i)
	{
		rcContour& contour = contourSet->conts[i];
		std::cout << " - - Contour " << i << " area: " << static_cast<int>(contour.area) << std::endl;
		cout << " - - - Number of verts:" << contourSet->conts[i].nrverts << endl;
		for (int v = 0; v < contourSet->conts[i].nrverts; v++)
		{
			cout << " - - - - Vert " << v << ": " << contourSet->conts[i].rverts[v * 4] << ", " << contourSet->conts[i].rverts[v * 4 + 1] << ", " << contourSet->conts[i].rverts[v * 4 + 2];
			cout << " > ";
			cout << "[" << originX + (contourSet->conts[i].rverts[v * 4] * cellSize) << ", " << originZ + (contourSet->conts[i].rverts[v * 4 + 2] * cellSize) << "]" << endl;
		}
	}

	rcPolyMesh* polyMesh = rcAllocPolyMesh();

	const int cMaxVertsPerPoly = 4;

	if (!rcBuildPolyMesh(&ctx, *contourSet, cMaxVertsPerPoly, *polyMesh))
	{
		cout << "Failed to build polymesh" << endl;
		rcFreePolyMesh(polyMesh);
		rcFreeContourSet(contourSet);
		return false;
	}

	rcFreeContourSet(contourSet);

	rcPolyMeshDetail* polyMeshDetail = rcAllocPolyMeshDetail();

	const float cSampleDistanceForHeightfieldinMetres = 0.5;
	const float cSampleMaxErrorForHeightfieldinMetres = 1.0;

	if (!rcBuildPolyMeshDetail(&ctx, *polyMesh, *compactHeightfield, cSampleDistanceForHeightfieldinMetres,
		cSampleMaxErrorForHeightfieldinMetres, *polyMeshDetail))
	{
		cout << "Failed to build polymesh detail" << endl;
		rcFreePolyMeshDetail(polyMeshDetail);
		rcFreePolyMesh(polyMesh);
		rcFreeCompactHeightfield(compactHeightfield);
		return false;
	}

	rcFreeCompactHeightfield(compactHeightfield);

	// Write to console the poly mesh details
	cout << "Poly mesh: " << endl;
	cout << " - Number of vertices: " << polyMesh->nverts << endl;
	cout << " - Number of polys: " << polyMesh->npolys << endl;
	// Write the polys to the console

	for (int p = 0; p < polyMesh->npolys; p++)
	{
		//Get poly as variable
		unsigned short* poly = &polyMesh->polys[p * 2 * polyMesh->nvp];


		cout << " - - Poly " << p << ": " << endl;
		cout << " - - - Verts: " << endl;
		// Write the verts as [x,y,z] to the console
		for (int v = 0; v < polyMesh->nvp; v++)
		{
			cout << " - - - Vert " << v << ": " << polyMesh->verts[poly[v]];
			cout << ", " << polyMesh->verts[poly[v + 1]];
			cout << ", " << polyMesh->verts[poly[v + 2]] << endl;
		}

		cout << " - - Flags: " << polyMesh->flags[p] << endl;
		cout << " - - Area: " << static_cast<int>(polyMesh->areas[p]) << endl;
	}


	//Write the verts to the console
	cout << "Poly mesh detail: " << endl;
	cout << " - Number of verts: " << polyMeshDetail->nverts << endl;
	for (int i = 0; i < polyMeshDetail->nverts; i++)
	{
		cout << " - - Vert " << i << ": " << polyMeshDetail->verts[i * 3] << ", " << polyMeshDetail->verts[i * 3 + 1] << ", " << polyMeshDetail->verts[i * 3 + 2] << endl;
	}

	dtNavMeshCreateParams params;
	memset(&params, 0, sizeof(params));
	//Add polymesh to the navmesh
	params.verts = polyMesh->verts;
	params.vertCount = polyMesh->nverts;
	params.polys = polyMesh->polys;
	params.polyFlags = polyMesh->flags;
	params.polyAreas = polyMesh->areas;
	params.polyCount = polyMesh->npolys;
	params.nvp = polyMesh->nvp;

	//Add general configuration to the navmesh
	params.walkableHeight = 2.0;
	params.walkableRadius = 0.6;
	params.walkableClimb = 0.9;
	params.cs = cCellSize;
	params.ch = cCellSize;
	params.buildBvTree = true;

	//Add polymesh detail to the navmesh is optional - skipping for now
	//Add off-mesh connections to the navmesh is optional - skipping for now

	//Create the navmesh
	unsigned char* navMeshData = 0;
	int navMeshDataSize = 0;

	if (!dtCreateNavMeshData(&params, &navMeshData, &navMeshDataSize))
	{
		cout << "Failed to create navmesh data" << endl;
		return false;
	}

	dtStatus status = navMesh_->init(navMeshData, navMeshDataSize, DT_TILE_FREE_DATA);

	rcFreePolyMeshDetail(polyMeshDetail);
	rcFreePolyMesh(polyMesh);

	if (dtStatusFailed(status))
	{
		cout << "Failed to init navmesh" << endl;
		dtFree(navMeshData);
		return false;
	}

	// Iterate through the navmesh tiles and polys and set the flags to the area




	for (int i = 0; i < navMesh_->getMaxTiles(); i++)
	{
		cout << "Tile " << i << endl;
		const dtMeshTile* tile = static_cast<const dtNavMesh*>(navMesh_)->getTile(i);
		if (tile == nullptr) {
			cout << "Failed to get tile by reference: " << i << endl;
			return false;
		}
		cout << "Number of polys: " << tile->header->polyCount << endl;

		for (int j = 0; j < tile->header->polyCount; j++)
		{
			cout << "Poly " << j << endl;
			dtPoly* poly = &tile->polys[j];
			unsigned char area = poly->getArea();
			unsigned short flag = static_cast<unsigned short>(area);
			poly->flags = flag;
			cout << "Poly " << j << " area: " << static_cast<int>(area) << " flag: " << poly->flags << endl;
		}
	}

	//Write to console the navmesh nodes
	cout << "Navmesh details: " << endl;

	cout << " - Number of tiles: " << navMesh_->getMaxTiles() << endl;

	//For each tile in the navmesh, write the tile details to the console
	for (int i = 0; i < navMesh_->getMaxTiles(); i++)
	{
		const dtMeshTile* tile = static_cast<const dtNavMesh*>(navMesh_)->getTile(i);
		cout << " - - Tile " << i << ": " << endl;
		cout << " - - - Number of verts: " << tile->header->vertCount << endl;
		cout << " - - - Number of links: " << tile->header->detailMeshCount << endl;
		cout << " - - - Number of polys: " << tile->header->polyCount << endl;

		for (int j = 0; j < tile->header->polyCount; j++)
		{
			const dtPoly poly = tile->polys[j];
			cout << " - - - - Poly " << j << ": " << endl;
			cout << " - - - - - Type: " << (static_cast<int>(poly.getType()) == DT_POLYTYPE_GROUND ? "DT_POLYTYPE_GROUND" :
				static_cast<int>(poly.getType()) == DT_POLYTYPE_OFFMESH_CONNECTION ? "DT_POLYTYPE_OFFMESH_CONNECTION" : "ERROR-UNKNOWN") << endl;
			cout << " - - - - - Area: " << static_cast<int>(poly.getArea()) << endl;
			cout << " - - - - - Flags: " << poly.flags << endl;
			cout << " - - - - - Vert base: " << poly.verts << endl;
			cout << " - - - - - Vert count: " << static_cast<int>(poly.vertCount) << endl;

			for (int v = 0; v < poly.vertCount; v++)
			{
				auto index = poly.verts[v];
				const float* vert = &tile->verts[index * 3];
				cout << " - - - - - - Vert " << v << "[" << index << "]: " << vert[0] << ", " << vert[1] << ", " << vert[2] << endl;
			}

			cout << " - - - - - Neighbour base: " << poly.neis << endl;
		}
	}

	status = navQuery_->init(navMesh_, 2048);

	if (dtStatusFailed(status))
	{
		cout << "Failed to init navmesh query" << endl;
		return false;
	}

	return true;
}

bool Mesh::RasterizeRectangle(rcContext& ctx, rcHeightfield& heightField, const std::array<float, 2>& bottomLeft, const std::array<float, 2>& topRight, const float& height)
{

	const float cObstacleVerts[12] = {
		bottomLeft[0], height, bottomLeft[1],
		topRight[0], height, bottomLeft[1],
		topRight[0], height, topRight[1],
		bottomLeft[0], height, topRight[1]
	};

	const unsigned short cObstacleTris[6] = {
		0, 1, 2,
		0, 2, 3
	};

	const unsigned char cTriAreaID[2] = { RC_WALKABLE_AREA, RC_WALKABLE_AREA };

	if (!rcRasterizeTriangles(&ctx, cObstacleVerts, 4, cObstacleTris, cTriAreaID, 2, heightField))
	{
		cout << "Failed to rasterize triangles" << endl;
		return false;
	}

	return true;
}

#pragma once
#include "Point.h"
#include <array>
#include <vector>
using MeshLib::CPoint;

class TetMeshInterFace
{
public:
	TetMeshInterFace() {};
	~TetMeshInterFace() {};

	std::vector<CPoint*> verts;
	std::vector<std::array<int, 4>> tetIDs;     // indices of 4 verts of each tets. The index is the index in verts;
	std::vector<int> visVerts;			   // indices of all surface verts. The index is the index in verts;
	std::vector<std::array<int, 3>> visTriIds;  // indices of 3 verts of each surface triangles. The index is the index in verts;

private:

};


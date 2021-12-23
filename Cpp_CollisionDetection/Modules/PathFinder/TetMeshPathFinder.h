#pragma once

#include <MeshFrame/core/Mesh/Iterators2.h>
#include <MeshFrame/core/Mesh/MeshCoreHeaders.h>
#include <MeshFrame/core/TetMesh/TMeshLibHeaders.h>
#include <MeshFrame/core/Geometry/Point4.h>

namespace PathFinder
{
	typedef MeshLib::CBaseMesh<MeshLib::CVertex, MeshLib::CEdge, MeshLib::CFace, MeshLib::CHalfEdge> M;
	typedef MeshLib::CIterators<M> It;
	using MeshLib::CPoint;

	namespace TMLib = MeshLib::TMeshLib;

	class CTetWithDestinationMark : public TMLib::CTet
	{
	public:
		CTetWithDestinationMark() : TMLib::CTet() {};
		~CTetWithDestinationMark()  {};

		bool isDestination = false;

	private:

	};

	

	typedef TMLib::CTMesh<TMLib::CTVertex, TMLib::CVertex, TMLib::CHalfEdge, TMLib::CTEdge, TMLib::CEdge, TMLib::CHalfFace, TMLib::CFace, CTetWithDestinationMark> TM;
	typedef TMLib::TIterators<TM> TIt;

	enum RayTargetPointIntersectionType
	{
		VertexInterSection,
		EdgeIntersection,
		FaceIntersection,

	};

	class TetMeshPathFinder
	{
	public:


		TetMeshPathFinder(M::Ptr inPM, TM::Ptr inPTetM);
		TetMeshPathFinder(TM::Ptr inPTetM);
		~TetMeshPathFinder();

		void tetMeshSurfaceMesh(std::vector<TM::VPtr>& verts, std::vector<TM::HFPtr>& faces);

		bool checkFaceFeasibleRegion(M::FPtr pF, const MeshLib::CPoint & p);
		bool checkEdgeFeasibleRegion(M::EPtr pE, const MeshLib::CPoint & p);
		bool checkVertexFeasibleRegion(M::VPtr pV, const MeshLib::CPoint & p);

		bool rayIntersectsTriangle(const CPoint& rayOrigin, const CPoint& rayVector, TM::HFPtr inTriangle,
			CPoint* outIntersectionPoint);

		// a function determing ray-triangle intersecion, though the difference with rayIntersectsTriangle is it output the intersection point as barycentrics
		bool rayIntersectsTriangleBrycentrics(const CPoint& rayOrigin, const CPoint& rayVector, TM::HFPtr inTriangle,
			MeshLib::CPoint4* outIntersectionPoint);

		bool meshVertexOnHF(M::VPtr pMeshV, TM::HFPtr surfaceHF);
		bool meshEdgeOnHF(M::EPtr pMeshE, TM::HFPtr surfaceHF);

		// return nullptr if cannot find a valid tet traverse to target point
		// return true if there is a valid traverse to pMeshClosestElement otherwise false
		bool rayTMeshTraverse(TM::TPtr pT, const CPoint& rayOrigin, const CPoint& rayVector, const CPoint& targetPoint,
			RayTargetPointIntersectionType intersectionTyep, void* pMeshClosestElement, std::vector<TM::TPtr>* traversedTVec);

		void markDesination(RayTargetPointIntersectionType intersectionTyep, void* pMeshClosestElement);
		void unmarkDesination(RayTargetPointIntersectionType intersectionTyep, void* pMeshClosestElement);

		M::Ptr pM;
		TM::Ptr pTetM;

		MeshLib::VPropHandle<TM::VPtr> meshVtoTVHandle;
		MeshLib::EPropHandle<TM::EPtr> meshEtoTEHandle;
		MeshLib::FPropHandle<TM::HFPtr> meshFtoTFHandle;
	private:
		const double rayTriIntersectionEPSILON = 1e-8;
		const double intersectionToTargetPointEPSILON = 1e-6;

	};



}



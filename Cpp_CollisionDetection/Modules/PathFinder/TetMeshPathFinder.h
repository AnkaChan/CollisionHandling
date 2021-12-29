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

	class TriIntersector
	{
	protected:
		MeshLib::CPoint org;
		double Sx, Sy, Sz;
		int   kx, ky, kz;
	public:
		void Init(const MeshLib::CPoint& rayDir, const MeshLib::CPoint& rayOrg)
		{
			org = rayOrg;

			// calculate dimension where the ray direction is maximal
			kz = maxComp(rayDir);
			kx = kz + 1; if (kx == 3) kx = 0;
			ky = kx + 1; if (ky == 3) ky = 0;

			// swap kx and ky dimension to preserve winding direction of triangles
			if (rayDir[kz] < 0.0f) {
				int temp = ky;
				ky = kx;
				kx = temp;
				//Swap(kx, ky);
			}

			// calculate shear constants
			Sx = rayDir[kx] / rayDir[kz];
			Sy = rayDir[ky] / rayDir[kz];
			Sz = 1.0 / rayDir[kz];
		}

		struct alignas(16) HitInfo { double u, v, w, t; };
		
		template<typename T, typename S> inline T    MultSign(T v, S sign) { return v * (sign < 0 ? T(-1) : T(1)); }	//!< Multiplies the given value with the given sign

		template <bool backfaceCulling = false>
		bool IntersectTriangle(HitInfo& hInfo, const MeshLib::CPoint& v0, const MeshLib::CPoint& v1, const MeshLib::CPoint& v2, double t_min = 0.0f, double t_max = DBL_MAX)
		{
			// calculate vertices relative to ray origin
			const MeshLib::CPoint A = v0 - org;
			const MeshLib::CPoint B = v1 - org;
			const MeshLib::CPoint C = v2 - org;

			// perform shear and scale of vertices
			const double Ax = A[kx] - Sx * A[kz];
			const double Ay = A[ky] - Sy * A[kz];
			const double Bx = B[kx] - Sx * B[kz];
			const double By = B[ky] - Sy * B[kz];
			const double Cx = C[kx] - Sx * C[kz];
			const double Cy = C[ky] - Sy * C[kz];

			// calculate scaled barycentric coordinates
			double U = Cx * By - Cy * Bx;
			double V = Ax * Cy - Ay * Cx;
			double W = Bx * Ay - By * Ax;

			//// debug only
			//hInfo.u = U;
			//hInfo.v = V;
			//hInfo.w = W;
			//hInfo.t = -1;

			// fallback to test against edges using double precision
			if (U == 0.0f || V == 0.0f || W == 0.0f) {
				double CxBy = Cx * By;
				double CyBx = Cy * Bx;
				U = (CxBy - CyBx);
				double AxCy = Ax * Cy;
				double AyCx = Ay * Cx;
				V = (AxCy - AyCx);
				double BxAy = Bx * Ay;
				double ByAx = By * Ax;
				W = (BxAy - ByAx);
			}

			// Perform edge tests. Moving this test before and at the end of the previous conditional gives higher performance.
			if (backfaceCulling) {
				if (U < 0.0f || V < 0.0f || W < 0.0f) return false;
			}
			else {
				if ((U < 0.0f || V < 0.0f || W < 0.0f) &&
					(U > 0.0f || V > 0.0f || W > 0.0f)) return false;
			}


			// calculate determinant
			double det = U + V + W;
			if (det == 0.0f) return false;

			// Calculate scaled z-coordinates of vertices and use them to calculate the hit distance.
			const double Az = Sz * A[kz];
			const double Bz = Sz * B[kz];
			const double Cz = Sz * C[kz];
			const double T = U * Az + V * Bz + W * Cz;
			
			//hInfo.t = T;

			if (backfaceCulling) {
				if (T < t_min || T > t_max * det) return false;
			}
			else {
				double abs_det = std::abs(det);
				if ((MultSign(T, det) < t_min * abs_det) ||
					(MultSign(T, det) > t_max * abs_det)) return false;
			}

			// normalize U, V, W, and T
			const double rcpDet = 1.0f / det;
			hInfo.u = U * rcpDet;
			hInfo.v = V * rcpDet;
			hInfo.w = W * rcpDet;
			hInfo.t = T * rcpDet;

			return true;
		}
	private:
		int maxComp(const MeshLib::CPoint& dir) {
			const double x = abs(dir[0]);
			const double y = abs(dir[1]);
			const double z = abs(dir[2]);

			if (x >= y && x >= z) {
				return 0;
			}
			else if (y >= x && y >= z)
			{
				return 1;
			}
			else
			{
				return 2;
			}
		}
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

		//bool rayIntersectsTriangle(const CPoint& rayOrigin, const CPoint& rayVector, TM::HFPtr inTriangle,
		//	CPoint* outIntersectionPoint);

		bool rayIntersectsTriangle(const CPoint& rayOrigin, const CPoint& rayVector, TM::HFPtr inTriangle, CPoint* outIntersectionPoint=nullptr);

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
		const double feasibleRegionEpsilon = -1e-7;

		const double rayTriIntersectionEPSILON = 1e-8;
		const double intersectionToTargetPointEPSILON = 1e-6;
		TriIntersector triIntersector;
		TriIntersector::HitInfo hitInfo;
		const double triIntersector_t_min = 0.;
	};



}



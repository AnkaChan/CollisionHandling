#include "TetMeshPathFinder.h"
#include "MeshFrame/core/Geometry/Point4.h"
#include <array>
#include <float.h>


namespace PathFinder
{

	TetMeshPathFinder::TetMeshPathFinder(M::Ptr inPM, TM::Ptr inPTetM) : pM(inPM), pTetM(inPTetM)
	{
	}

	TetMeshPathFinder::TetMeshPathFinder(TM::Ptr inPTetM) : pTetM(inPTetM)
	{
		std::vector<TM::VPtr> surfaceVerts;
		std::vector<TM::HFPtr> surfaceFaces;

		tetMeshSurfaceMesh(surfaceVerts, surfaceFaces);

		std::vector<std::array<double, 3>> surfaceVertsCoords;
		std::vector<std::array<int, 3>> surfaceFacesVIds;
		std::vector<int> vIds;

		//TMLib::VPropHandle<bool> vAddedHandle;
		//pTetM->addVProp(vAddedHandle);
		//for (TM::VPtr pV : TIt::TM_VIterator(pTetM)) {
		//	pTetM->gVP(vAddedHandle, pV) = false;
		//}

		for (TM::VPtr pV: surfaceVerts) {
			vIds.push_back(pV->id());
			std::array<double, 3> pt = {pV->position()[0], pV->position()[1], pV->position()[2]};
			surfaceVertsCoords.push_back(std::move(pt));

			//pTetM->gVP(vAddedHandle, pV) = true;
		}

		for (size_t iF = 0; iF < surfaceFaces.size(); iF++)
		{
			std::array<int, 3> fVIds;
			int fV = 0;
			for (TM::VPtr pFV : TIt::HF_VIterator(surfaceFaces[iF])) {
				fVIds[fV] = pFV->id();
				//assert(pTetM->gVP(vAddedHandle, pFV));
				//auto idPos = std::find(vIds.begin(), vIds.end(), fVIds[fV]);
				//assert(idPos != vIds.end());
				++fV;

			}

			surfaceFacesVIds.push_back(fVIds);
		}

		pM = new M;
		pM->readVFList(&surfaceVertsCoords, &surfaceFacesVIds, &vIds);
		
		// link surface mesh Vertex and Face to TetMesh
		pM->addVProp(meshVtoTVHandle);
		pM->addFProp(meshFtoTFHandle);
		pM->addEProp(meshEtoTEHandle);

		int vIndex = 0;
		for (M::VPtr pV : It::MVIterator(pM)) {
			assert(pV->id() == surfaceVerts[vIndex]->id());

			pM->gVP(meshVtoTVHandle, pV) = surfaceVerts[vIndex];
			++vIndex;
		}

		int fIndex = 0;
		for (M::FPtr pF : It::MFIterator(pM)) {
			int fVIndex = 0;
			for (M::VPtr pFV : It::FVIterator(pF)) {
				assert(pFV->id() == surfaceFacesVIds[fIndex][fVIndex]);
				++fVIndex;
			}

			pM->gFP(meshFtoTFHandle, pF) = surfaceFaces[fIndex];
			++fIndex;

		}

		for (M::EPtr pE : It::MEIterator(pM))
		{
			TM::VPtr pV1 = pM->gVP(meshVtoTVHandle, M::edgeVertex1(pE));
			TM::VPtr pV2 = pM->gVP(meshVtoTVHandle, M::edgeVertex2(pE));
			pM->getEProp(meshEtoTEHandle, pE) = pTetM->VertexEdge(pV1, pV2);
			assert(pM->getEProp(meshEtoTEHandle, pE) != nullptr);

		}

	}

	bool TetMeshPathFinder::lineIntersectsTriangle(const CPoint& rayOrigin, const CPoint& rayVector, TM::HFPtr inTriangle
		, TriIntersector& triIntersector, TriIntersector::HitInfo& hitInfo, MeshLib::CPoint* outIntersectionPoint)
	{
		TM::HEPtr pHE = TM::HalfFaceHalfEdge(inTriangle);
		const CPoint& v0 = TM::HalfEdgeSource(pHE)->position();
		const CPoint& v1 = TM::HalfEdgeTarget(pHE)->position();
		const CPoint& v2 = TM::HalfEdgeTarget(TM::HalfEdgeNext(pHE))->position();

		bool hadsIntersection = triIntersector.IntersectTriangle(hitInfo, v0, v1, v2, -DBL_MAX);
		if (outIntersectionPoint != nullptr && hadsIntersection) {
			*outIntersectionPoint = rayOrigin + rayVector * hitInfo.t;
		}
		return hadsIntersection;

	}
	
	bool TetMeshPathFinder::rayIntersectsTriangle(const CPoint & rayOrigin, const CPoint & rayVector, TM::HFPtr inTriangle, 
		TriIntersector& triIntersector, TriIntersector::HitInfo& hitInfo, MeshLib::CPoint* outIntersectionPoint)
	{
		TM::HEPtr pHE = TM::HalfFaceHalfEdge(inTriangle);
		const CPoint & v0 = TM::HalfEdgeSource(pHE)->position();
		const CPoint & v1 = TM::HalfEdgeTarget(pHE)->position();
		const CPoint & v2 = TM::HalfEdgeTarget(TM::HalfEdgeNext(pHE))->position();

		bool hadsIntersection= triIntersector.IntersectTriangle(hitInfo, v0, v1, v2, triIntersector_t_min);
		if (outIntersectionPoint != nullptr && hadsIntersection) {
			*outIntersectionPoint = rayOrigin + rayVector * hitInfo.t;
		}
		return hadsIntersection;

		//CPoint edge1, edge2, h, s, q;
		//double a, f, u, v;
		//edge1 = triangleVerts[1] - triangleVerts[0];
		//edge2 = triangleVerts[2] - triangleVerts[0];
		//h = rayVector ^ edge2;
		//a = edge1*h;
		//if (a > - rayTriIntersectionEPSILON && a < rayTriIntersectionEPSILON)
		//	return false;    // This ray is parallel to this triangle.

		//f = 1.0 / a;
		//s = rayOrigin - triangleVerts[0];
		//u = f * s*h;
		////if (v < 0.0 || u + v > 1.0)
		//if (u < -rayTriIntersectionEPSILON || u > 1.0 + rayTriIntersectionEPSILON)
		//	return false;
		//q = s^edge1;
		//v = f * rayVector*q;
		////if (v < 0.0 || u + v > 1.0)
		//if (v < -rayTriIntersectionEPSILON || u + v > 1.0 + rayTriIntersectionEPSILON)
		//	return false;
		//// At this stage we can compute t to find out where the intersection point is on the line.
		//double t = f * edge2*q;
		//if (t > rayTriIntersectionEPSILON) // ray intersection
		//{
		//	if (outIntersectionPoint != nullptr) {
		//		*outIntersectionPoint = rayOrigin + rayVector * t;
		//	}
		//	return true;
		//}
		//else // This means that there is a line intersection but not a ray intersection.
		//	//printf("Ray Edge intersection detected!\n");
		//	return false;
	}

	// a function determing ray-triangle intersecion, though the difference with rayIntersectsTriangle is it output the intersection point as barycentrics
	//bool TetMeshPathFinder::rayIntersectsTriangleBrycentrics(const CPoint& rayOrigin, const CPoint& rayVector, TM::HFPtr inTriangle,
	//	MeshLib::CPoint4* outIntersectionPoint)
	//{
	//	CPoint triangleVerts[3];
	//	TM::HalfFace3Points(inTriangle, triangleVerts);

	//	CPoint edge1, edge2, h, s, q;
	//	double a, f, u, v;
	//	edge1 = triangleVerts[1] - triangleVerts[0];
	//	edge2 = triangleVerts[2] - triangleVerts[0];
	//	h = rayVector ^ edge2;
	//	a = edge1 * h;
	//	if (a > -rayTriIntersectionEPSILON && a < rayTriIntersectionEPSILON) {
	//		if(outIntersectionPoint != nullptr) {
	//			(*outIntersectionPoint)[0] = -1.;
	//			(*outIntersectionPoint)[1] = -1.;
	//			(*outIntersectionPoint)[2] = -1.;
	//			(*outIntersectionPoint)[3] = -1.;
	//		}
	//		return false;    // This ray is parallel to this triangle.
	//	}
	//	f = 1.0 / a;
	//	s = rayOrigin - triangleVerts[0];
	//	u = f * s * h;
	//	//if (v < 0.0 || u + v > 1.0)
	//	if (u < -rayTriIntersectionEPSILON || u > 1.0 + rayTriIntersectionEPSILON) {
	//		if (outIntersectionPoint != nullptr) {
	//			(*outIntersectionPoint)[0] = u;
	//			(*outIntersectionPoint)[1] = -1.;
	//			(*outIntersectionPoint)[2] = -1.;
	//			(*outIntersectionPoint)[3] = -1.;
	//		}
	//		return false;
	//	}
	//	q = s ^ edge1;
	//	v = f * rayVector * q;
	//	//if (v < 0.0 || u + v > 1.0)
	//	if (v < -rayTriIntersectionEPSILON || u + v > 1.0 + rayTriIntersectionEPSILON) {
	//		if (outIntersectionPoint != nullptr) {
	//			(*outIntersectionPoint)[0] = u;
	//			(*outIntersectionPoint)[1] = v;
	//			(*outIntersectionPoint)[2] = -1.;
	//			(*outIntersectionPoint)[3] = -1.;
	//		}
	//		return false;
	//	}
	//	// At this stage we can compute t to find out where the intersection point is on the line.
	//	double t = f * edge2 * q;
	//	if (outIntersectionPoint != nullptr) {
	//		(*outIntersectionPoint)[0] = u;
	//		(*outIntersectionPoint)[1] = v;
	//		(*outIntersectionPoint)[2] = 1.0 - u - v;
	//		(*outIntersectionPoint)[3] = t;
	//	}

	//	if (t > rayTriIntersectionEPSILON) // ray intersection
	//	{

	//		return true;
	//	}
	//	else // This means that there is a line intersection but not a ray intersection.
	//		//printf("Ray Edge intersection detected!\n");
	//		return false;
	//}

	// a function determing ray-triangle intersecion, though the difference with rayIntersectsTriangle is it output the intersection point as barycentrics
	// not water tight
	//bool TetMeshPathFinder::rayIntersectsTriangleBrycentrics(const CPoint& rayOrigin, const CPoint& rayVector, TM::HFPtr inTriangle,
	//	MeshLib::CPoint4* outIntersectionPoint)
	//{
	//	CPoint triangleVerts[3];
	//	TM::HalfFace3Points(inTriangle, triangleVerts);

	//	CPoint edge1, edge2, h, s, q;
	//	double a, f, u, v;
	//	edge1 = triangleVerts[1] - triangleVerts[0];
	//	edge2 = triangleVerts[2] - triangleVerts[0];
	//	h = rayVector ^ edge2;
	//	a = edge1 * h;
	//	if (a > -rayTriIntersectionEPSILON && a < rayTriIntersectionEPSILON) {
	//		if (outIntersectionPoint != nullptr) {
	//			(*outIntersectionPoint)[0] = -1.;
	//			(*outIntersectionPoint)[1] = -1.;
	//			(*outIntersectionPoint)[2] = -1.;
	//			(*outIntersectionPoint)[3] = -1.;
	//		}
	//		return false;    // This ray is parallel to this triangle.
	//	}
	//	f = 1.0 / a;
	//	s = rayOrigin - triangleVerts[0];
	//	u = f * s * h;
	//	//if (v < 0.0 || u + v > 1.0)
	//	if (u < -rayTriIntersectionEPSILON || u > 1.0 + rayTriIntersectionEPSILON) {
	//		if (outIntersectionPoint != nullptr) {
	//			(*outIntersectionPoint)[0] = u;
	//			(*outIntersectionPoint)[1] = -1.;
	//			(*outIntersectionPoint)[2] = -1.;
	//			(*outIntersectionPoint)[3] = -1.;
	//		}
	//		return false;
	//	}
	//	q = s ^ edge1;
	//	v = f * rayVector * q;
	//	//if (v < 0.0 || u + v > 1.0)
	//	if (v < -rayTriIntersectionEPSILON || u + v > 1.0 + rayTriIntersectionEPSILON) {
	//		if (outIntersectionPoint != nullptr) {
	//			(*outIntersectionPoint)[0] = u;
	//			(*outIntersectionPoint)[1] = v;
	//			(*outIntersectionPoint)[2] = -1.;
	//			(*outIntersectionPoint)[3] = -1.;
	//		}
	//		return false;
	//	}
	//	// At this stage we can compute t to find out where the intersection point is on the line.
	//	double t = f * edge2 * q;
	//	if (outIntersectionPoint != nullptr) {
	//		(*outIntersectionPoint)[0] = u;
	//		(*outIntersectionPoint)[1] = v;
	//		(*outIntersectionPoint)[2] = 1.0 - u - v;
	//		(*outIntersectionPoint)[3] = t;
	//	}

	//	if (t > rayTriIntersectionEPSILON) // ray intersection
	//	{

	//		return true;
	//	}
	//	else // This means that there is a line intersection but not a ray intersection.
	//		//printf("Ray Edge intersection detected!\n");
	//		return false;
	//}

	// a function determing ray-triangle intersecion, though the difference with rayIntersectsTriangle is it output the intersection point as barycentrics
	// not water tight
	bool TetMeshPathFinder::rayIntersectsTriangleBrycentrics(const CPoint& rayOrigin, const CPoint& rayVector, TM::HFPtr inTriangle,
		MeshLib::CPoint4* outIntersectionPoint)
	{
		CPoint triangleVerts[3];
		TM::HalfFace3Points(inTriangle, triangleVerts);

		CPoint edge1, edge2, h, s, q;
		double a, f, u, v;
		edge1 = triangleVerts[1] - triangleVerts[0];
		edge2 = triangleVerts[2] - triangleVerts[0];
		h = rayVector ^ edge2;
		a = edge1 * h;
		if (a > -rayTriIntersectionEPSILON && a < rayTriIntersectionEPSILON) {
			if (outIntersectionPoint != nullptr) {
				(*outIntersectionPoint)[0] = -1.;
				(*outIntersectionPoint)[1] = -1.;
				(*outIntersectionPoint)[2] = -1.;
				(*outIntersectionPoint)[3] = -1.;
			}
			return false;    // This ray is parallel to this triangle.
		}
		f = 1.0 / a;
		s = rayOrigin - triangleVerts[0];
		u = f * s * h;
		//if (v < 0.0 || u + v > 1.0)
		if (u < -rayTriIntersectionEPSILON || u > 1.0 + rayTriIntersectionEPSILON) {
			if (outIntersectionPoint != nullptr) {
				(*outIntersectionPoint)[0] = u;
				(*outIntersectionPoint)[1] = -1.;
				(*outIntersectionPoint)[2] = -1.;
				(*outIntersectionPoint)[3] = -1.;
			}
			return false;
		}
		q = s ^ edge1;
		v = f * rayVector * q;
		//if (v < 0.0 || u + v > 1.0)
		if (v < -rayTriIntersectionEPSILON || u + v > 1.0 + rayTriIntersectionEPSILON) {
			if (outIntersectionPoint != nullptr) {
				(*outIntersectionPoint)[0] = u;
				(*outIntersectionPoint)[1] = v;
				(*outIntersectionPoint)[2] = -1.;
				(*outIntersectionPoint)[3] = -1.;
			}
			return false;
		}
		// At this stage we can compute t to find out where the intersection point is on the line.
		double t = f * edge2 * q;
		if (outIntersectionPoint != nullptr) {
			(*outIntersectionPoint)[0] = u;
			(*outIntersectionPoint)[1] = v;
			(*outIntersectionPoint)[2] = 1.0 - u - v;
			(*outIntersectionPoint)[3] = t;
		}

		if (t > rayTriIntersectionEPSILON) // ray intersection
		{

			return true;
		}
		else // This means that there is a line intersection but not a ray intersection.
			//printf("Ray Edge intersection detected!\n");
			return false;
	}

	//bool TetMeshPathFinder::rayTMeshTraverse(TM::TPtr pT, const CPoint& rayOrigin, const CPoint& rayVector, const CPoint& targetPoint,
	//	RayTargetPointIntersectionType intersectionTyep, void* pMeshClosestElement, std::vector<TM::TPtr>* traversedTVec)
	//{
	//	bool hasIntersection = false;
	//	TM::HFPtr pIntersectedHF= nullptr;
	//	CPoint intersectionPoint;

	//	if (traversedTVec !=  nullptr)
	//	{
	//		traversedTVec->push_back(pT);
	//	}
	//	for ( TM::HFPtr pHF : TIt::T_HFIterator(pT))
	//	{
	//		hasIntersection = rayIntersectsTriangle(rayOrigin, rayVector, pHF, &intersectionPoint);
	//		if (hasIntersection) {
	//			pIntersectedHF = pHF;
	//			
	//			break;
	//		}
	//	}

	//	assert(pIntersectedHF != nullptr);
	//	
	//	while (true)
	//	{
	//		// If intersectionPoint have reached the targetPoint, determine if the closest element is the final halfface
	//		if ((targetPoint - intersectionPoint).norm() <= intersectionToTargetPointEPSILON)
	//		{
	//			switch (intersectionTyep)
	//			{
	//			case PathFinder::VertexInterSection:
	//				if (meshVertexOnHF(M::VPtr(pMeshClosestElement), pIntersectedHF)) 
	//					return true;
	//				else
	//					return false;
	//				break;
	//			case PathFinder::EdgeIntersection:
	//				if (meshEdgeOnHF(M::EPtr(pMeshClosestElement), pIntersectedHF)) 
	//					return true;
	//				else
	//					return false;
	//				break;
	//			case PathFinder::FaceIntersection:
	//				if (pM->gFP(meshFtoTFHandle, M::FPtr(pMeshClosestElement)) == pIntersectedHF)
	//					return true;
	//				else
	//					return false;
	//				break;
	//			default:
	//				break;
	//			}
	//		}
	//		else
	//		{
	//			// If intersectionPoint have not reached the targetPoint, traverse to the next tet
	//			TM::HFPtr pHFDual = TM::HalfFaceDual(pIntersectedHF);
	//			if (pHFDual == nullptr) {
	//					// the traverse stopped before reaching targetPoint
	//					return false;
	//			}

	//			if ((intersectionPoint - targetPoint) * rayVector > rayTriIntersectionEPSILON) {
	//				// the traverse has passed through target point, in this case this isn't a valid tet traverse
	//				return false;
	//			}

	//			// find the face where the ray came out this tet
	//			if (traversedTVec != nullptr)
	//			{
	//				traversedTVec->push_back(TM::HalfFaceTet(pHFDual));
	//			}

	//			hasIntersection = false;

	//			for (TM::HEPtr pHE : TIt::HF_HEIterator(pHFDual)) {
	//				TM::HFPtr pHF = pHE->dual()->half_face();
	//				hasIntersection = rayIntersectsTriangle(rayOrigin, rayVector, pHF, &intersectionPoint);
	//				if (hasIntersection) {
	//					pIntersectedHF = pHF;
	//					break;
	//				}
	//			}

	//			if (!hasIntersection) {
	//				printf("Critical problem encountered!\n");
	//				printf("No intersection found after pass through a triangle!\n");

	//				printf("Prior face traversed:\n");
	//				for (TM::VPtr pV : TIt::HF_VIterator(pHFDual)) {
	//					printf("%f %f %f\n", pV->position()[0], pV->position()[1], pV->position()[2]);
	//				}
	//				printf("Intersection Point: %f %f %f\n", intersectionPoint[0], intersectionPoint[1], intersectionPoint[2]);

	//				MeshLib::CPoint4 baryCentrics;
	//				rayIntersectsTriangleBrycentrics(rayOrigin, rayVector, pIntersectedHF, &baryCentrics);
	//				printf("Intersection BaryCentrics: %f %f %f %f\n", baryCentrics[0], baryCentrics[1], baryCentrics[2], baryCentrics[3]);

	//				printf("Candidate Faces:\n");
	//				for (TM::HEPtr pHE : TIt::HF_HEIterator(pHFDual)) {
	//					printf("Face:\n");
	//					TM::HFPtr pHF = pHE->dual()->half_face();
	//					for (TM::VPtr pV : TIt::HF_VIterator(pHF)) {
	//						printf("%f %f %f\n", pV->position()[0], pV->position()[1], pV->position()[2]);
	//					}
	//					
	//					hasIntersection = rayIntersectsTriangleBrycentrics(rayOrigin, rayVector, pHF, &baryCentrics);
	//					printf("Intersection BaryCentrics: %f %f %f %f\n", baryCentrics[0], baryCentrics[1], baryCentrics[2], baryCentrics[3]);
	//				}
	//				

	//				assert(false);
	//				getchar();

	//			}
	//		}
	//		}


	//	// have reached the boundary

	//	return false;
	//}

	bool TetMeshPathFinder::rayTMeshTraverse(TM::TPtr pT, const CPoint& rayOrigin, const CPoint& rayVector, const MeshLib::CPoint& targetPoint,
		RayTargetPointIntersectionType intersectionTyep, void* pMeshClosestElement, std::vector<TM::TPtr>* traversedTVec)
	{
		TriIntersector triIntersector;
		TriIntersector::HitInfo hitInfo;

		triIntersector.Init(rayVector, rayOrigin);

		TM::TPtr pCurrentT = pT;
		if (pCurrentT->isDestination)
		{
			return true;
		}

		bool hasIntersection = false;
		TM::HFPtr pIntersectedHF = nullptr;
		CPoint intersectionPoint;

		if (traversedTVec != nullptr)
		{
			traversedTVec->push_back(pT);
		}
		for (TM::HFPtr pHF : TIt::T_HFIterator(pT))
		{
			hasIntersection = rayIntersectsTriangle(rayOrigin, rayVector, pHF, triIntersector, hitInfo, &intersectionPoint);
			//printf("HitInfo: %f %f %f %f\n", hitInfo.u, hitInfo.v, hitInfo.w, hitInfo.t);

			if (hasIntersection) {
				pIntersectedHF = pHF;

				break;
			}
		}

		assert(pIntersectedHF != nullptr);

		while (true)
		{
			// If intersectionPoint have not reached the targetPoint, traverse to the next tet
			TM::HFPtr pHFDual = TM::HalfFaceDual(pIntersectedHF);
			if (pHFDual == nullptr) {
				// the traverse stopped before reaching targetPoint
				return false;
			}

			if ((intersectionPoint - targetPoint) * rayVector > rayTriIntersectionEPSILON) {
				// the traverse has passed through target point, in this case this isn't a valid tet traverse
				return false;
			}

			pCurrentT = TM::HalfFaceTet(pHFDual);
			// find the face where the ray came out this tet
			if (traversedTVec != nullptr)
			{
				traversedTVec->push_back(pCurrentT);
			}

			// if 
			if (pCurrentT->isDestination)
			{
				return true;
			}

			hasIntersection = false;

			for (TM::HEPtr pHE : TIt::HF_HEIterator(pHFDual)) {
				TM::HFPtr pHF = pHE->dual()->half_face();
				hasIntersection = rayIntersectsTriangle(rayOrigin, rayVector, pHF, triIntersector, hitInfo, &intersectionPoint);
				if (hasIntersection) {
					pIntersectedHF = pHF;
					break;
				}
			}

			if (!hasIntersection) {
				printf("Critical problem encountered!\n");
				printf("No intersection found after pass through a triangle!\n");

				printf("Prior face traversed:\n");
				for (TM::VPtr pV : TIt::HF_VIterator(pHFDual)) {
					printf("%f %f %f\n", pV->position()[0], pV->position()[1], pV->position()[2]);
				}
				printf("Intersection Point: %f %f %f\n", intersectionPoint[0], intersectionPoint[1], intersectionPoint[2]);

				MeshLib::CPoint4 baryCentrics;
				rayIntersectsTriangleBrycentrics(rayOrigin, rayVector, pIntersectedHF, &baryCentrics);
				printf("Intersection BaryCentrics: %f %f %f %f\n", baryCentrics[0], baryCentrics[1], baryCentrics[2], baryCentrics[3]);

				printf("Candidate Faces:\n");
				for (TM::HEPtr pHE : TIt::HF_HEIterator(pHFDual)) {
					printf("Face:\n");
					TM::HFPtr pHF = pHE->dual()->half_face();
					for (TM::VPtr pV : TIt::HF_VIterator(pHF)) {
						printf("%f %f %f\n", pV->position()[0], pV->position()[1], pV->position()[2]);
					}

					hasIntersection = rayIntersectsTriangleBrycentrics(rayOrigin, rayVector, pHF, &baryCentrics);
					printf("Intersection BaryCentrics: %f %f %f %f\n", baryCentrics[0], baryCentrics[1], baryCentrics[2], baryCentrics[3]);
				}

				assert(false);
				getchar();

			}
		}


		// have reached the boundary

		return false;
	}

	bool TetMeshPathFinder::rayTMeshTraverseSurfaceToQueryTet(TM::TPtr pDestinationTet, TM::HFPtr pStartingHF, const CPoint& rayOrigin, const CPoint& rayVector, const CPoint& queryPoint, 
		RayTargetPointIntersectionType intersectionTyep, void* pMeshClosestElement, std::vector<TM::TPtr>* traversedTVec)
	{
		
		TriIntersector triIntersector;
		TriIntersector::HitInfo hitInfo;

		triIntersector.Init(rayVector, rayOrigin);

		TM::TPtr pCurrentT = nullptr;
		bool hasIntersection = false;
		//assert(TM::PointInTet(TM::HalfFaceTet(pStartingHF), rayOrigin));

		TM::HFPtr pIntersectedHF = pStartingHF;
		CPoint intersectionPoint(0., 0., 0.);
		while (true)
		{
			hasIntersection = false;
			pCurrentT = TM::HalfFaceTet(pIntersectedHF);
			// find the face where the ray came out this tet
			if (traversedTVec != nullptr)
			{
				traversedTVec->push_back(pCurrentT);
			}
			if (pCurrentT == pDestinationTet)
			{
				return true;
			}

			for (TM::HEPtr pHE : TIt::HF_HEIterator(pIntersectedHF)) {
				TM::HFPtr pHF = pHE->dual()->half_face();
				if (lineIntersectsTriangle(rayOrigin, rayVector, pHF, triIntersector, hitInfo, &intersectionPoint)) {
					pIntersectedHF = pHF;
					hasIntersection = true;

					break;
				}
			}
			if (!hasIntersection) {
				printf("Critical problem encountered for Tet: %d!\n", pCurrentT->id());
				printf("No intersection found after pass through a triangle!\n");

				printf("Prior face traversed:\n");
				for (TM::VPtr pV : TIt::HF_VIterator(pIntersectedHF)) {
					printf("%f %f %f\n", pV->position()[0], pV->position()[1], pV->position()[2]);
				}
				printf("Intersection Point: %f %f %f\n", intersectionPoint[0], intersectionPoint[1], intersectionPoint[2]);

				MeshLib::CPoint4 baryCentrics;
				rayIntersectsTriangleBrycentrics(rayOrigin, rayVector, pIntersectedHF, &baryCentrics);
				printf("Intersection BaryCentrics: %f %f %f %f\n", baryCentrics[0], baryCentrics[1], baryCentrics[2], baryCentrics[3]);

				printf("Candidate Faces:\n");
				for (TM::HEPtr pHE : TIt::HF_HEIterator(pIntersectedHF)) {
					printf("Face:\n");
					TM::HFPtr pHF = pHE->dual()->half_face();
					for (TM::VPtr pV : TIt::HF_VIterator(pHF)) {
						printf("%f %f %f\n", pV->position()[0], pV->position()[1], pV->position()[2]);
					}

					hasIntersection = rayIntersectsTriangleBrycentrics(rayOrigin, rayVector, pHF, &baryCentrics);
					printf("Intersection BaryCentrics: %f %f %f %f\n", baryCentrics[0], baryCentrics[1], baryCentrics[2], baryCentrics[3]);
				}
				getchar();
				assert(false);

				return false;

			}

			// If intersectionPoint have not reached the targetPoint, traverse to the next tet
			pIntersectedHF = TM::HalfFaceDual(pIntersectedHF);
			if (pIntersectedHF == nullptr) {
				// the traverse stopped before reaching targetPoint
				return false;
			}

			if ((intersectionPoint - queryPoint) * rayVector > rayTriIntersectionEPSILON) {
				// the traverse has passed through target point, in this case this isn't a valid tet traverse
				return false;
			}

		}

		return false;
	}

	void TetMeshPathFinder::markDesination(RayTargetPointIntersectionType intersectionTyep, void* pMeshClosestElement)
	{
		TM::TPtr pT = nullptr;
		switch (intersectionTyep)
		{
		case PathFinder::VertexInterSection:
			for (TM::TVPtr pTv : TIt::V_TVIterator(pM->gVP(meshVtoTVHandle, M::VPtr(pMeshClosestElement))))
			{
				pT = TM::TVertexTet(pTv);
				pT->isDestination = true;
			}
			break;
		case PathFinder::EdgeIntersection:
			for (TM::TEPtr pTe : TIt::E_TEIterator(pM->gEP(meshEtoTEHandle, M::EPtr(pMeshClosestElement))))
			{
				pT = TM::TEdgeTet(pTe);
				pT->isDestination = true;
			}
			break;
		case PathFinder::FaceIntersection:
			pT = TM::HalfFaceTet(pM->gFP(meshFtoTFHandle, M::FPtr(pMeshClosestElement)));
			pT->isDestination = true;
			break;
		default:
			break;
		}
	}

	void TetMeshPathFinder::unmarkDesination(RayTargetPointIntersectionType intersectionTyep, void* pMeshClosestElement)
	{
		TM::TPtr pT = nullptr;
		switch (intersectionTyep)
		{
		case PathFinder::VertexInterSection:
			for (TM::TVPtr pTv : TIt::V_TVIterator(pM->gVP(meshVtoTVHandle, M::VPtr(pMeshClosestElement))))
			{
				pT = TM::TVertexTet(pTv);
				pT->isDestination = false;
			}
			break;
		case PathFinder::EdgeIntersection:
			for (TM::TEPtr pTe : TIt::E_TEIterator(pM->gEP(meshEtoTEHandle, M::EPtr(pMeshClosestElement))))
			{
				pT = TM::TEdgeTet(pTe);
				pT->isDestination = false;
			}
			break;
		case PathFinder::FaceIntersection:
			pT = TM::HalfFaceTet(pM->gFP(meshFtoTFHandle, M::FPtr(pMeshClosestElement)));
			pT->isDestination = false;
			break;
		default:

			break;
		}
	}

	TetMeshPathFinder::~TetMeshPathFinder()
	{
		delete pM;
	}

	void TetMeshPathFinder::tetMeshSurfaceMesh(std::vector<TM::VPtr>& verts, std::vector<TM::HFPtr>& faces)
	{
		verts.clear();
		faces.clear();

		std::list<TM::HFPtr> surfaceHFList;
		auto pVless = [](TM::VPtr pVa, TM::VPtr pVb) {
			if (pVa->id() < pVb->id())
				return true;
			else
				return false;
		};
		std::set<TM::VPtr, decltype(pVless)> vSet(pVless);
		for (TM::HFPtr pHF : TIt::TM_HFIterator(pTetM)) {
			if (TM::HalfFaceDual(pHF) == NULL) {
				surfaceHFList.push_back(pHF);
				for (auto pV : TIt::HF_VIterator(pHF)) {
					vSet.insert(pV);
				}
			}
		}

		std::copy(vSet.begin(), vSet.end(), std::back_inserter(verts));
		std::copy(surfaceHFList.begin(), surfaceHFList.end(), std::back_inserter(faces));
	}

	void TetMeshPathFinder::updateSurfaceMesh()
	{
		for (M::VPtr pV : It::MVIterator(pM))
		{
			pV->point() = pM->gVP(meshVtoTVHandle, pV)->position();
		}
	}

	bool TetMeshPathFinder::checkFaceFeasibleRegion(M::FPtr pF, const MeshLib::CPoint& p)
	{
		CPoint fNormal = pM->faceOrientedArea(pF);

		M::HEPtr pHE = pM->faceHalfedge(pF);
		M::HEPtr pHENext = pM->halfedgeNext(pHE);

		const CPoint & A =  pHE->target()->point();
		const CPoint & B = pHENext->target()->point();
		const CPoint & C = pHE->source()->point(); 

		CPoint AP = p - A;

		if (AP * fNormal > -feasibleRegionEpsilon) {
			return false;
		}

		CPoint AB = B - A;
		CPoint nAB = fNormal ^ AB;

		if (AP * nAB <= feasibleRegionEpsilon) {
			return false;
		}

		CPoint BC = C - B;
		CPoint nBC = fNormal ^ BC;
		CPoint BP = p - B;

		if (BP * nBC <= feasibleRegionEpsilon) {
			return false;
		}

		CPoint CA = A - C;
		CPoint nCA = fNormal ^ CA;
		if (AP * nCA <= feasibleRegionEpsilon) {
			return false;
		}

		return true;
	}

	bool TetMeshPathFinder::checkEdgeFeasibleRegion(M::EPtr pE, const MeshLib::CPoint& p)
	{
		M::FPtr pF1 = pM->edgeFace1(pE);
		M::FPtr pF2 = pM->edgeFace2(pE);

		if (pF1 == nullptr || pF2 == nullptr) {
			assert(false);
			printf("Boundary edge encountered! The mesh is supposed to be water tight.\n");
			getchar();
		}

		CPoint fNormal1 = pM->faceOrientedArea(pF1);
		CPoint fNormal2 = pM->faceOrientedArea(pF2);

		const CPoint& A = pM->edgeVertex1(pE)->point();
		const CPoint& B = pM->edgeVertex2(pE)->point();
		CPoint AP = p - A;

		CPoint AB = B - A;

		if (AP * AB <= feasibleRegionEpsilon) {
			return false;
		}

		CPoint BP = p - B;

		if (BP * -AB <= feasibleRegionEpsilon) {
			return false;
		}

		CPoint nAB = AB ^ fNormal1;
		if (AP * nAB <= feasibleRegionEpsilon) {
			return false;
		}

		CPoint nBA = (-AB) ^ fNormal2;
		if (AP * nBA <= feasibleRegionEpsilon) {
			return false;
		}

		return true;
	}

	bool TetMeshPathFinder::checkVertexFeasibleRegion(M::VPtr pV, const MeshLib::CPoint& p)
	{
		const CPoint& A = pV->point();

		CPoint AP = p - A;
		for (M::VPtr pVNei : It::VVIterator(pV)) {
			CPoint BA = A - pVNei->point();

			if (AP * BA <= feasibleRegionEpsilon) {
				return false;
			}
		}

		return true;

	}



	bool TetMeshPathFinder::meshVertexOnHF(M::VPtr pMeshV, TM::HFPtr surfaceHF) {
		for (TM::VPtr pTMeshV : TIt::HF_VIterator(surfaceHF)) {
			if (pM->gVP(meshVtoTVHandle, pMeshV) == pTMeshV) {
				return true;
			}
		}
		return false;
	}


	bool TetMeshPathFinder::meshEdgeOnHF(M::EPtr pMeshE, TM::HFPtr surfaceHF) {
		for (TM::HEPtr pTMeshHe : TIt::HF_HEIterator(surfaceHF)) {
			TM::VPtr pMeshV1 = pM->gVP(meshVtoTVHandle, M::edgeVertex1(pMeshE));
			TM::VPtr pMeshV2 = pM->gVP(meshVtoTVHandle, M::edgeVertex2(pMeshE));

			TM::VPtr pTMeshV1 = TM::HalfEdgeSource(pTMeshHe);
			TM::VPtr pTMeshV2 = TM::HalfEdgeTarget(pTMeshHe);

			if ((pMeshV1 == pTMeshV1 && pMeshV2 == pTMeshV2) ||
				(pMeshV1 == pTMeshV2 && pMeshV2 == pTMeshV1))
			{
				return true;
			}
		}
		return false;
	}
}
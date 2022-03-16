#include "PointQuery.h"

//#define TBB_PARALLEL

#ifdef TBB_PARALLEL
#include "oneapi/tbb.h"

#endif // TBB_PARALLEL


using namespace embree;
using namespace PathFinder;

void tetBoundsFunc(const struct RTCBoundsFunctionArguments* args)
{
    const TM::Ptr pTMesh = (const TM::Ptr)args->geometryUserPtr;
    const TM::TContainer & rContainter = pTMesh->tets();

    RTCBounds* bounds_o = args->bounds_o;

    const TM::TType & t = rContainter[args->primID];

    const CPoint& p1 = t.vertex(0)->position();
    const CPoint& p2 = t.vertex(1)->position();
    const CPoint& p3 = t.vertex(2)->position();
    const CPoint& p4 = t.vertex(3)->position();

    bounds_o->lower_x = min(min(min(p1[0], p2[0]), p3[0]), p4[0]);
    bounds_o->lower_y = min(min(min(p1[1], p2[1]), p3[1]), p4[1]);
    bounds_o->lower_z = min(min(min(p1[2], p2[2]), p3[2]), p4[2]);
    bounds_o->upper_x = max(max(max(p1[0], p2[0]), p3[0]), p4[0]);
    bounds_o->upper_y = max(max(max(p1[1], p2[1]), p3[1]), p4[1]);
    bounds_o->upper_z = max(max(max(p1[2], p2[2]), p3[2]), p4[2]);
}

embree::PointQuery::PointQuery(const std::vector<TM::Ptr>& inMeshPtrs)
    : meshPtrs(inMeshPtrs)
    , tMeshPtrs(std::move(inMeshPtrs))
{
    device = rtcNewDevice(NULL);

    numTetsTotal = 0;
    for (TM::Ptr pTM : meshPtrs)
    {
        RTCScene scene = rtcNewScene(device);
        RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

        TetMeshPathFinder* pathFinder = new TetMeshPathFinder(pTM);
        pathFinders.push_back(pathFinder);
        pathFinder->pM->reinitializeVId();
        pathFinder->pM->reinitializeFId();

        float* vb = getVBuffer(pathFinder->pM, geom);
        unsigned int* ib = getFBuffer(pathFinder->pM, geom);

        rtcSetGeometryPointQueryFunction(geom, closestPointFunc);

        rtcCommitGeometry(geom);
        unsigned int geomId = rtcAttachGeometry(scene, geom);
        rtcReleaseGeometry(geom);
        rtcCommitScene(scene);

        vbs.push_back(vb);
        ibs.push_back(ib);
        meshGeometryIds.push_back(geomId);
        scenes.push_back(scene);

        numTetsTotal += pTM->numTets();
    }

    // add all the tet mesh to a single scene for collision detection
    tetMeshesScene = rtcNewScene(device);

    unsigned int tMId = 0;
    for (TM::Ptr pTM : meshPtrs)
    {
        RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER);
        rtcSetGeometryUserPrimitiveCount(geom, pTM->numTets());
        rtcSetGeometryBoundsFunction(geom, tetBoundsFunc, nullptr);
        rtcSetGeometryUserData(geom, (void*)pTM);
        
        rtcSetGeometryPointQueryFunction(geom, tetIntersectionFunc);
        rtcCommitGeometry(geom);
        unsigned int geomId = rtcAttachGeometry(tetMeshesScene, geom);
        tetmeshGeoIdToPointerMap[geomId] = pTM;
        tetmeshPtrToTetMeshIndexMap[pTM] = tMId;
        tMId++;
        tetmeshGeometryIds.push_back(geomId);
        rtcReleaseGeometry(geom);
    }
    rtcCommitScene(tetMeshesScene);

}

embree::PointQuery::~PointQuery()
{

    for (TetMeshPathFinder* pathFinder : pathFinders) {
        delete pathFinder;
    }
    for (auto scene : scenes) {
        rtcReleaseScene(scene);
    }

    rtcReleaseDevice(device);
}

float* embree::PointQuery::getVBuffer(M::Ptr pM, RTCGeometry geom)
{
    size_t numVerts = pM->numVertices();

    float * vb = (float*)rtcSetNewGeometryBuffer(geom,
        RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float), numVerts);


    for (M::VPtr pV : It::MVIterator(pM))
    {
        vb[pV->id() * 3] = pV->point()[0];
        vb[pV->id() * 3 + 1] = pV->point()[1];
        vb[pV->id() * 3 + 2] = pV->point()[2];
    }

    return vb;
}

unsigned int* embree::PointQuery::getFBuffer(M::Ptr pM, RTCGeometry geom)
{
    size_t numFaces = pM->numFaces();

    unsigned* ib = (unsigned*)rtcSetNewGeometryBuffer(geom,
        RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned), numFaces);

    for (M::FPtr pF : It::MFIterator(pM))
    {
        int iFV = 0;

        for (M::VPtr pVF : It::FVIterator(pF))
        {
            ib[pF->id() * 3 + iFV] = pVF->id();
            iFV += 1;
        }

    }
    return ib;
}

bool embree::PointQuery::queryPoint(ClosestPointResult& result, const CPoint& p, PathFinder::TM::TPtr pEmbraceTet, unsigned int sceneId, float radius,
    bool checkFeasibleRegion, bool checkTetTraverse, std::vector<PathFinder::TM::TPtr>* traversedTVec)
{
    RTCPointQuery query;
    query.x = p[0];
    query.y = p[1];
    query.z = p[2];
    query.radius = radius;
    query.time = 0.f;

    result.primID = RTC_INVALID_GEOMETRY_ID;
    result.geomID = RTC_INVALID_GEOMETRY_ID;
    result.pathFinder = pathFinders[sceneId];
    result.checkFeasibleRegion = checkFeasibleRegion;
    result.checkTetTraverse = checkTetTraverse;
    result.nFaceTraversed = 0;
    result.pEmbraceTet = pEmbraceTet;
    result.d = inf;
    result.traversedTVec = traversedTVec;
    result.closestP[0] = 0.0;
    result.closestP[1] = 0.0;
    result.closestP[2] = 0.0;

    RTCPointQueryContext context;
    rtcInitPointQueryContext(&context);
    rtcPointQuery(scenes[sceneId], &query, &context, nullptr, (void*)&result);

    if (result.primID != RTC_INVALID_GEOMETRY_ID && result.geomID != RTC_INVALID_GEOMETRY_ID) {
        return true;
    }
    else
    {
        return false;
    }
}


Vec3fa embree::PointQuery::closestPointTriangle(Vec3fa const& p, Vec3fa const& a, Vec3fa const& b, Vec3fa const& c, ClosestPointOnTriangleType& pointType)
{
    const Vec3fa ab = b - a;
    const Vec3fa ac = c - a;
    const Vec3fa ap = p - a;

    const float d1 = dot(ab, ap);
    const float d2 = dot(ac, ap);
    if (d1 <= 0.f && d2 <= 0.f) {
        pointType = AtA;
        return a;
    }

    const Vec3fa bp = p - b;
    const float d3 = dot(ab, bp);
    const float d4 = dot(ac, bp);
    if (d3 >= 0.f && d4 <= d3) {
        pointType = AtB;
        return b;
    }

    const Vec3fa cp = p - c;
    const float d5 = dot(ab, cp);
    const float d6 = dot(ac, cp);
    if (d6 >= 0.f && d5 <= d6) {
        pointType = AtC;
        return c;
    }

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
    {
        const float v = d1 / (d1 - d3);
        pointType = AtAB;
        return a + v * ab;
    }

    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
    {
        const float v = d2 / (d2 - d6);
        pointType = AtAC;
        return a + v * ac;
    }

    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)
    {
        pointType = AtBC;
        const float v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + v * (c - b);
    }

    const float denom = 1.f / (va + vb + vc);
    const float v = vb * denom;
    const float w = vc * denom;
    pointType = AtInterior;
    return a + v * ab + w * ac;
}

PathFinder::CPoint embree::PointQuery::closestPointTriangle(PathFinder::CPoint const& p, PathFinder::CPoint const& a, PathFinder::CPoint const& b,
    PathFinder::CPoint const& c, ClosestPointOnTriangleType& pointType) {
    const PathFinder::CPoint ab = b - a;
    const PathFinder::CPoint ac = c - a;
    const PathFinder::CPoint ap = p - a;

    const double d1 = ab * ap;
    const double d2 = ac * ap;
    if (d1 <= 0.f && d2 <= 0.f) {
        pointType = AtA;
        return a;
    }

    const PathFinder::CPoint bp = p - b;
    const double d3 = ab * bp;
    const double d4 = ac * bp;
    if (d3 >= 0.f && d4 <= d3) {
        pointType = AtB;
        return b;
    }

    const PathFinder::CPoint cp = p - c;
    const double d5 = ab * cp;
    const double d6 = ac * cp;
    if (d6 >= 0.f && d5 <= d6) {
        pointType = AtC;
        return c;
    }

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
    {
        const double v = d1 / (d1 - d3);
        pointType = AtAB;
        return a + v * ab;
    }

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
    {
        const double v = d2 / (d2 - d6);
        pointType = AtAC;
        return a + v * ac;
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)
    {
        pointType = AtBC;
        const double v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + v * (c - b);
    }

    const double denom = 1.f / (va + vb + vc);
    const double v = vb * denom;
    const double w = vc * denom;
    pointType = AtInterior;
    return a + v * ab + w * ac;
}

bool embree::PointQuery::vertexCollisionDetection(PathFinder::TM::VPtr pTMV, PathFinder::TM::Ptr pTM, CollisionResult* pResult)
{
    RTCPointQueryContext context;
    rtcInitPointQueryContext(&context);
    const CPoint& p = pTMV->position();
    RTCPointQuery query;
    query.x = p[0];
    query.y = p[1];
    query.z = p[2];
    query.radius = 0.f;
    query.time = 0.f;

    pResult->pTMQuery = pTM;
    pResult->pVQuery = pTMV;
    pResult->pTetmeshGeoIdToPointerMap = &tetmeshGeoIdToPointerMap;

    rtcPointQuery(tetMeshesScene, &query, &context, nullptr, (void*) pResult);

    return false;
}

bool embree::PointQuery::vertexCollisionDetectionAndClosestPointQuery(PathFinder::TM::VPtr pTMV, PathFinder::TM::Ptr pTM, 
    CollisionResult* pColResult, bool checkFeasibleRegion, bool checkTetTraverse)
{
    vertexCollisionDetection(pTMV, pTM, pColResult);
    const CPoint& p = pTMV->position();
    ClosestPointResult closestPtResult;

    for (size_t iIntersection = 0; iIntersection < pColResult->intersectedTets.size(); iIntersection++)
    {
        std::vector<PathFinder::TM::TPtr>* traverseVecs = nullptr;
        if (pColResult->pTetTraversed != nullptr) {
            pColResult->pTetTraversed->push_back(std::vector<PathFinder::TM::TPtr>());
            traverseVecs = &pColResult->pTetTraversed->back();
        }

        TM::Ptr pTMIntersected = pColResult->intersectedTMeshes[iIntersection];
        TM::TPtr pTetIntersected = pColResult->intersectedTets[iIntersection];

        unsigned int surfaceSceneId = tetmeshPtrToTetMeshIndexMap[pTMIntersected];
        queryPoint(closestPtResult, p, pTetIntersected, surfaceSceneId, inf, checkFeasibleRegion, checkTetTraverse, traverseVecs);

        // for testing
        //queryPoint(closestPtResult, p, pTetIntersected, surfaceSceneId, inf);
        //CPoint closestP1 = closestPtResult.closestP;
        //queryPoint(closestPtResult, p, pTetIntersected, surfaceSceneId, inf, false, false);
        //CPoint closestP2 = closestPtResult.closestP;

        //assert((closestP2 - closestP1).norm() < 1e-6);

        pColResult->closestPoints.push_back(closestPtResult.closestP);
    }

    return false;
}

void embree::PointQuery::update(const std::vector<int>* pVecTMeshIds)
{
    for (size_t i = 0; i < (*pVecTMeshIds).size(); i++)
    {
        int tMeshId = (*pVecTMeshIds)[i];
        // get the tet geom buffer
        unsigned int geoId = tetmeshGeometryIds[tMeshId];
        TM::Ptr pTM = tMeshPtrs[tMeshId];
        RTCGeometry geom = rtcGetGeometry(tetMeshesScene, geoId);
        rtcUpdateGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0);
        rtcCommitGeometry(geom);

        // upate surface Mesh
        pathFinders[tMeshId]->updateSurfaceMesh();
        RTCScene surfaceScene = scenes[tMeshId];
        RTCGeometry geomSurface = rtcGetGeometry(surfaceScene, meshGeometryIds[tMeshId]);
        float* vb = (float*)rtcGetGeometryBufferData(geomSurface, RTC_BUFFER_TYPE_VERTEX, 0);

        for (M::VPtr pV : It::MVIterator(pathFinders[tMeshId]->pM))
        {
            vb[pV->id() * 3] = pV->point()[0];
            vb[pV->id() * 3 + 1] = pV->point()[1];
            vb[pV->id() * 3 + 2] = pV->point()[2];
        }

        rtcUpdateGeometryBuffer(geomSurface, RTC_BUFFER_TYPE_VERTEX, 0);
        rtcCommitGeometry(geomSurface);
        rtcCommitScene(surfaceScene);
    }
    rtcCommitScene(tetMeshesScene);



    //#ifdef TBB_PARALLEL
    //    tbb::parallel_for(0, (int)(*pVecTMeshIds).size(), [&](int i)
    //        {
    //
    //
    //
    //        }
    //    );
    //
    //
    //
    //#endif // TBB_PARALLEL


}

bool embree::tetIntersectionFunc(RTCPointQueryFunctionArguments* args)
{
    CollisionResult* result = (CollisionResult*)args->userPtr;

    TM::Ptr pTM = (*(result->pTetmeshGeoIdToPointerMap))[args->geomID];

    assert(args->userPtr);
    const unsigned int geomID = args->geomID;
    const unsigned int primID = args->primID;

    RTCPointQueryContext* context = args->context;
    const unsigned int stackSize = args->context->instStackSize;
    const unsigned int stackPtr = stackSize - 1;

    TM::TType& t = pTM->tets()[args->primID];
    if (pTM == result->pTMQuery) {
        if (result->pVQuery == (TM::VPtr)t.vertex(0) || 
            result->pVQuery == (TM::VPtr)t.vertex(1) || 
            result->pVQuery == (TM::VPtr)t.vertex(2) || 
            result->pVQuery == (TM::VPtr)t.vertex(3) )
        {
            return false;
        }
    }

    CPoint qq(args->query->x, args->query->y, args->query->z);
    if (TM::PointInTet(&t, qq)) {
        result->intersectedTets.push_back(&t);
        result->intersectedTMeshes.push_back(pTM);
    }
    return false;

}

bool embree::closestPointFunc(RTCPointQueryFunctionArguments* args)
{
    ClosestPointResult* result = (ClosestPointResult*)args->userPtr;
    
    TetMeshPathFinder* pathFinder = result->pathFinder;
    M::Ptr m = pathFinder->pM;


    assert(args->userPtr);
    const unsigned int geomID = args->geomID;
    const unsigned int primID = args->primID;

    RTCPointQueryContext* context = args->context;
    const unsigned int stackSize = args->context->instStackSize;
    const unsigned int stackPtr = stackSize - 1;

    AffineSpace3fa inst2world = stackSize > 0
        ? (*(AffineSpace3fa*)context->inst2world[stackPtr])
        : one;

    //// query position in world space
    //Vec3fa q(args->query->x, args->query->y, args->query->z);
    //PathFinder::CPoint qq(args->query->x, args->query->y, args->query->z);

    ///*
    // * Get triangle information in local space
    // */

    //M::FPtr pF = &(m->getFContainer()[primID]);

    //Vec3fa vs[3];

    //M::VPtr vPtrs[3] = { nullptr, nullptr, nullptr, };
    //M::EPtr ePtrs[3] = { nullptr, nullptr, nullptr, };

    //int iFV = 0;
    //for (M::HEPtr pHE : It::FHEIterator(pF))
    //{
    //    M::VPtr pVF = pHE->source();
    //    vs[iFV].x = pVF->point()[0];
    //    vs[iFV].y = pVF->point()[1];
    //    vs[iFV].z = pVF->point()[2];

    //    vPtrs[iFV] = pVF;
    //    ePtrs[iFV] = pHE->edge();

    //    iFV += 1;

    //}

    ///*
    // * Determine distance to closest point on triangle (implemented in
    // * common/math/closest_point.h), and transform in world space if necessary.
    // */
    //ClosestPointOnTriangleType pointType;
    //const Vec3fa p = PointQuery::closestPointTriangle(q, vs[0], vs[1], vs[2], pointType);
    //const PathFinder::CPoint closestP(p.x, p.y, p.z);
    //float d = distance(q, p);
    //// printf_s("Queried triangle with distance: %f\n", d);

     // query position in world space
    PathFinder::CPoint qq(args->query->x, args->query->y, args->query->z);

    /*
     * Get triangle information in local space
     */

    M::FPtr pF = &(m->getFContainer()[primID]);

    PathFinder::CPoint vs[3];

    M::VPtr vPtrs[3] = { nullptr, nullptr, nullptr, };
    M::EPtr ePtrs[3] = { nullptr, nullptr, nullptr, };

    int iFV = 0;
    for (M::HEPtr pHE : It::FHEIterator(pF))
    {
        M::VPtr pVF = pHE->source();
        vs[iFV] = pVF->point();

        vPtrs[iFV] = pVF;
        ePtrs[iFV] = pHE->edge();

        iFV += 1;

    }

    /*
     * Determine distance to closest point on triangle (implemented in
     * common/math/closest_point.h), and transform in world space if necessary.
     */
    ClosestPointOnTriangleType pointType;
    const PathFinder::CPoint closestP = PointQuery::closestPointTriangle(qq, vs[0], vs[1], vs[2], pointType);
    float d = (qq - closestP).norm();
    // printf_s("Queried triangle with distance: %f\n", d);


    if (args->similarityScale > 0)
        d = d / args->similarityScale;

    /*
     * Store result in userPtr and update the query radius if we found a point
     * closer to the query position. This is optional but allows for faster
     * traversal (due to better culling).
     */
    if (d < args->query->radius)
    {
        PathFinder::RayTargetPointIntersectionType intersectionType;
        if (result->checkFeasibleRegion)
        {
            bool inFeasibleRegion = false;

            switch (pointType)
            {
            case AtInterior:
                inFeasibleRegion = result->pathFinder->checkFaceFeasibleRegion(pF, qq);
                result->pMeshClosestElement = pF;
                intersectionType = PathFinder::FaceIntersection;
                break;
            case AtAB:
                inFeasibleRegion = result->pathFinder->checkEdgeFeasibleRegion(ePtrs[0], qq);
                result->pMeshClosestElement = ePtrs[0];
                intersectionType = PathFinder::EdgeIntersection;
                break;
            case AtBC:
                inFeasibleRegion = result->pathFinder->checkEdgeFeasibleRegion(ePtrs[1], qq);
                result->pMeshClosestElement = ePtrs[1];
                intersectionType = PathFinder::EdgeIntersection;
                break;
            case AtAC:
                inFeasibleRegion = result->pathFinder->checkEdgeFeasibleRegion(ePtrs[2], qq);
                result->pMeshClosestElement = ePtrs[2];
                intersectionType = PathFinder::EdgeIntersection;
                break;
            case AtA:
                inFeasibleRegion = result->pathFinder->checkVertexFeasibleRegion(vPtrs[0], qq);
                result->pMeshClosestElement = vPtrs[0];
                intersectionType = PathFinder::VertexInterSection;
                break;
            case AtB:
                inFeasibleRegion = result->pathFinder->checkVertexFeasibleRegion(vPtrs[1], qq);
                result->pMeshClosestElement = vPtrs[1];
                intersectionType = PathFinder::VertexInterSection;
                break;
            case AtC:
                inFeasibleRegion = result->pathFinder->checkVertexFeasibleRegion(vPtrs[2], qq);
                result->pMeshClosestElement = vPtrs[2];
                intersectionType = PathFinder::VertexInterSection;
                break;
            case NotFound:
                inFeasibleRegion = false;
                result->pMeshClosestElement = nullptr;
                break;
            default:
                break;
            }

            if (!inFeasibleRegion) {
                return false;
            }
            else if (result->checkTetTraverse) {

                // query point traverse to closest point 
                //PathFinder::CPoint rayDirection = (closestP - qq);
                //pathFinder->markDesination(intersectionType, result->pMeshClosestElement);
                //bool hasValidTraverse = pathFinder->rayTMeshTraverse(result->pEmbraceTet, qq, rayDirection, closestP, intersectionType, 
                //   result->pMeshClosestElement, result->traversedTVec );
                //pathFinder->unmarkDesination(intersectionType, result->pMeshClosestElement);

                // closest point traverse to query point 
                PathFinder::CPoint closestPTracing = closestP + 0.01 * ((vs[0] + vs[1] + vs[2]) / 3.0 - closestP);

                PathFinder::CPoint rayDirection = (qq - closestPTracing);
                rayDirection = rayDirection / rayDirection.norm();
                TM::HFPtr pStartingHF = pathFinder->pM->gFP(pathFinder->meshFtoTFHandle, pF);
                /*std::cout << vs[0] << "\n" << vs[1] << "\n" << vs[2] << std::endl;
                for (TM::VPtr pTetV : TIt::HF_VIterator(pStartingHF))
                {
                    std::cout << pTetV->position() << "\n";
                }*/
                if (result->traversedTVec != nullptr)
                {
                    result->traversedTVec->clear();
                }

                bool hasValidTraverse = pathFinder->rayTMeshTraverseSurfaceToQueryTet(result->pEmbraceTet, pStartingHF, closestPTracing, rayDirection, qq, intersectionType,
                       result->pMeshClosestElement, result->traversedTVec );

                if (!hasValidTraverse) {
                    return false;
                }
            }
        }
        else
        {
            intersectionType = FaceIntersection;
        }
        result->nFaceTraversed++;

        args->query->radius = d;
        result->primID = primID;
        result->geomID = geomID;
        result->d = d;
        result->closestP = closestP;
        result->intersectionType = intersectionType;
        return true; // Return true to indicate that the query radius changed.
    }


    return false;
}

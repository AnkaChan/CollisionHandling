#include "PointQuery.h"

using namespace embree;
using namespace PathFinder;

embree::PointQuery::PointQuery(const std::vector<TM::Ptr>& inMeshPtrs)
    : meshPtrs(inMeshPtrs)
{
    device = rtcNewDevice(NULL);

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


    }

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

bool embree::PointQuery::queryPoint(const CPoint& p, PathFinder::TM::TPtr pEmbraceTet, unsigned int sceneId, float radius, 
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


    // query position in world space
    Vec3fa q(args->query->x, args->query->y, args->query->z);
    PathFinder::CPoint qq(args->query->x, args->query->y, args->query->z);

    /*
     * Get triangle information in local space
     */

    M::FPtr pF = &(m->getFContainer()[primID]);

    Vec3fa vs[3];

    M::VPtr vPtrs[3] = { nullptr, nullptr, nullptr, };
    M::EPtr ePtrs[3] = { nullptr, nullptr, nullptr, };

    int iFV = 0;
    for (M::HEPtr pHE : It::FHEIterator(pF))
    {
        M::VPtr pVF = pHE->source();
        vs[iFV].x = pVF->point()[0];
        vs[iFV].y = pVF->point()[1];
        vs[iFV].z = pVF->point()[2];

        vPtrs[iFV] = pVF;
        ePtrs[iFV] = pHE->edge();

        iFV += 1;

    }

    /*
     * Determine distance to closest point on triangle (implemented in
     * common/math/closest_point.h), and transform in world space if necessary.
     */
    ClosestPointOnTriangleType pointType;
    const Vec3fa p = PointQuery::closestPointTriangle(q, vs[0], vs[1], vs[2], pointType);
    const PathFinder::CPoint closestP(p.x, p.y, p.z);
    float d = distance(q, p);
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

                PathFinder::CPoint rayDirection = (closestP - qq);
                rayDirection = rayDirection / rayDirection.norm();

                pathFinder->markDesination(intersectionType, result->pMeshClosestElement);
                bool hasValidTraverse = pathFinder->rayTMeshTraverse(result->pEmbraceTet, qq, rayDirection, closestP, intersectionType, 
                   result->pMeshClosestElement, result->traversedTVec );
                pathFinder->unmarkDesination(intersectionType, result->pMeshClosestElement);


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

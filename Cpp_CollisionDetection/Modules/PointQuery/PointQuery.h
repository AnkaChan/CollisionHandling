#pragma once

#include <embree3/rtcore.h>
#include "../common/math/vec2.h"
#include "../common/math/vec3.h"
#include "../common/math/vec4.h"
#include "../common/math/affinespace.h"
#include "../PathFinder/TetMeshPathFinder.h"
#include "MeshFrame/core/Memory/Array.h"


namespace embree {

	struct Geometry
	{

	};

    enum ClosestPointOnTriangleType
    {
        AtA,
        AtB,
        AtC,
        AtAB,
        AtBC,
        AtAC,
        AtInterior,
        NotFound
    };

    struct ClosestPointResult
    {
        ClosestPointResult()
            : primID(RTC_INVALID_GEOMETRY_ID)
            , geomID(RTC_INVALID_GEOMETRY_ID)
        {}

        PathFinder::CPoint closestP;

        unsigned int primID;
        unsigned int geomID;
        float d;
        bool checkFeasibleRegion = false;
        bool checkTetTraverse = false;
        int nFaceTraversed = 0;
        PathFinder::TetMeshPathFinder* pathFinder = nullptr;
        PathFinder::RayTargetPointIntersectionType intersectionType;
        std::vector<PathFinder::TM::TPtr>* traversedTVec = nullptr;

        void* pMeshClosestElement;
        PathFinder::TM::TPtr pEmbraceTet;
    };

    struct CollisionResult
    {
        CollisionResult()
        {}

        PathFinder::TM::VPtr pVQuery;
        PathFinder::TM::Ptr pTMQuery;
        CPArray<PathFinder::TM::TPtr, 8> intersectedTets;
        CPArray<PathFinder::TM::Ptr, 8> intersectedTMeshes;
        CPArray<PathFinder::CPoint, 8> closestPoints;
        std::map<unsigned int, PathFinder::TM::Ptr>* pTetmeshGeoIdToPointerMap;
        std::vector<std::vector<PathFinder::TM::TPtr>>* pTetTraversed = nullptr;

    };

    bool closestPointFunc(RTCPointQueryFunctionArguments* args);
    bool tetIntersectionFunc(RTCPointQueryFunctionArguments* args);

	class PointQuery 
    {
    public:
        PointQuery(const std::vector<PathFinder::TM::Ptr> & inTMeshPtrs);
        ~PointQuery();

        float* getVBuffer(PathFinder::M::Ptr pM, RTCGeometry geom);
        unsigned int* getFBuffer(PathFinder::M::Ptr pM, RTCGeometry geom);

        bool queryPoint(ClosestPointResult &closestPtResult, const PathFinder::CPoint & p, PathFinder::TM::TPtr pT, unsigned int sceneId, float radius = inf,
            bool checkFeasibleRegion=true, bool checkTetTraverse=true, std::vector<PathFinder::TM::TPtr>* traversedTVec = nullptr);

        int nFaceTraversed = 0;
        static Vec3fa closestPointTriangle(Vec3fa const& p, Vec3fa const& a, Vec3fa const& b, Vec3fa const& c, ClosestPointOnTriangleType& pointType);
        static PathFinder::CPoint closestPointTriangle(PathFinder::CPoint const& p, PathFinder::CPoint const& a, PathFinder::CPoint const& b,
            PathFinder::CPoint const& c, ClosestPointOnTriangleType& pointType);

        bool vertexCollisionDetection(PathFinder::TM::VPtr pTMV, PathFinder::TM::Ptr pTM, CollisionResult* pResult);

        bool vertexCollisionDetectionAndClosestPointQuery(PathFinder::TM::VPtr pTMV, PathFinder::TM::Ptr pTM, CollisionResult* pResult, bool checkFeasibleRegion = true, bool checkTetTraverse = true);

        void update(const std::vector<int> * pVecTMeshIds);

        // Scences for each surface mesh
        std::vector<PathFinder::TM::Ptr> meshPtrs;
        std::vector<unsigned int> meshGeometryIds;
        std::vector<RTCScene> scenes; // a scene for each geometry
        std::vector<PathFinder::TetMeshPathFinder* > pathFinders;

        // tet mesh status
        std::vector<PathFinder::TM::Ptr> tMeshPtrs;
        int numTetsTotal;
        RTCScene tetMeshesScene;
        std::vector<unsigned int> tetmeshGeometryIds;
        std::map<unsigned int, PathFinder::TM::Ptr> tetmeshGeoIdToPointerMap;
        std::map<PathFinder::TM::Ptr, unsigned int> tetmeshPtrToTetMeshIndexMap;

        std::vector<float*> vbs;
        std::vector<unsigned int*> ibs;


        RTCDevice device;


	};



}

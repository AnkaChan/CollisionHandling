#pragma once

#include <embree3/rtcore.h>
#include "../common/math/vec2.h"
#include "../common/math/vec3.h"
#include "../common/math/vec4.h"
#include "../common/math/affinespace.h"
#include "../PathFinder/TetMeshPathFinder.h"


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

    bool closestPointFunc(RTCPointQueryFunctionArguments* args);

	class PointQuery 
    {
    public:
        PointQuery(const std::vector<PathFinder::TM::Ptr> & inTMeshPtrs);
        ~PointQuery();

        float* getVBuffer(PathFinder::M::Ptr pM, RTCGeometry geom);
        unsigned int* getFBuffer(PathFinder::M::Ptr pM, RTCGeometry geom);

        bool queryPoint(const PathFinder::CPoint & p, PathFinder::TM::TPtr pT, unsigned int sceneId, float radius = inf, 
            bool checkFeasibleRegion=true, bool checkTetTraverse=true, std::vector<PathFinder::TM::TPtr>* traversedTVec = nullptr);


        int nFaceTraversed = 0;
        static Vec3fa closestPointTriangle(Vec3fa const& p, Vec3fa const& a, Vec3fa const& b, Vec3fa const& c, ClosestPointOnTriangleType& pointType);

        std::vector<PathFinder::TM::Ptr> meshPtrs;
        std::vector<unsigned int> meshGeometryIds;
        std::vector<RTCScene> scenes; // a scene for each geometry
        std::vector<PathFinder::TetMeshPathFinder* > pathFinders;

        std::vector<float*> vbs;
        std::vector<unsigned int*> ibs;

        RTCDevice device;
        ClosestPointResult result;
	};



}

#include <limits>
#include <iostream>
#include <stdlib.h>
#include <ctime>

#include "PointQuery/PointQuery.h"
#include "PathFinder/TetMeshPathFinder.h"
// #include <random>


using namespace  embree;
using namespace  PathFinder;

M::Ptr m;

void shiftTetMesh(TM& tm, CPoint shiftDir) {
    for (TM::VPtr pV : TIt::TM_VIterator(&tm))
    {
        pV->position() = pV->position() + shiftDir;
    }
}
    
int main()
{
    // testPathFinder();
    TM tetM1;
    tetM1._load_t("C:/Code/02_Graphics/CollisionHandling/Cpp_CollisionDetection/Data/bun_10KV20KF.t", true);

    TM tetM2;
    tetM2._load_t("C:/Code/02_Graphics/CollisionHandling/Cpp_CollisionDetection/Data/bun_10KV20KF.t", true);
    shiftTetMesh(tetM2, CPoint(0.05, 0., 0.));

    std::vector<TM::Ptr> tmeshes = { &tetM1, &tetM2 };
    PointQuery pointQuery(tmeshes);

    for (TM::VPtr pV : TIt::TM_VIterator(&tetM2))
    {
        CollisionResult * pColResult = new CollisionResult;
        //pointQuery.vertexCollisionDetection(pV, &tetM2, pColResult);
        pointQuery.vertexCollisionDetectionAndClosestPointQuery(pV, &tetM2, pColResult);

        if (pColResult->intersectedTets.size() != 0)
        {
            assert(pColResult->intersectedTets.size() == 1);
            assert(pColResult->intersectedTMeshes[0] == &tetM1);

            std::cout << "Intersection found! " << "Point " << pV->position() << " Intersected with " << pColResult->intersectedTets.size() << "Points!"
                << "Closest point is on: " << pColResult->closestPoints[0] << "\n";

        }

        delete pColResult;
    }




}
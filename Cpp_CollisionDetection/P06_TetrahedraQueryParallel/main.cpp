#include <limits>
#include <iostream>
#include <stdlib.h>
#include <ctime>

#include "PointQuery/PointQuery.h"
#include "PathFinder/TetMeshPathFinder.h"
// #include <random>
#include "oneapi/tbb.h"

using namespace  embree;  
using namespace  PathFinder;

M::Ptr m;

void shiftTetMesh(TM& tm, CPoint shiftDir) {
    for (TM::VPtr pV : TIt::TM_VIterator(&tm))
    {
        pV->position() = pV->position() + shiftDir;
    }
}
    
int main(){
    //tbb::parallel_for(0, 100, [&](int i) { printf("%d", i); });
    //return 0;
    // testPathFinder();

    TM tetM1;
    tetM1._load_t("C:/Code/02_Graphics/CollisionHandling/Cpp_CollisionDetection/Data/bun_10KV20KF.t", true);

    TM tetM2;
    tetM2._load_t("C:/Code/02_Graphics/CollisionHandling/Cpp_CollisionDetection/Data/bun_10KV20KF.t", true);
    shiftTetMesh(tetM2, CPoint(0.10, 0., 0.));

    printf("Number of tetrahedra: %d\n", tetM2.tets().size());

    std::vector<TM::Ptr> tmeshes = { &tetM1, &tetM2 };
    PointQuery pointQuery(tmeshes);

    size_t timeTries = 1;

    clock_t timeStart_single = clock();
    // single threaded
    for (TM::VPtr pV : TIt::TM_VIterator(&tetM2))
    {
        for (size_t k = 0; k < timeTries; k++)
        {
            CollisionResult* pColResult = new CollisionResult;
            //pointQuery.vertexCollisionDetection(pV, &tetM2, pColResult);
            pointQuery.vertexCollisionDetectionAndClosestPointQuery(pV, &tetM2, pColResult);

            if (pColResult->intersectedTets.size() != 0)
            {
                assert(pColResult->intersectedTets.size() == 1);
                assert(pColResult->intersectedTMeshes[0] == &tetM1);

                //std::cout << "Intersection found! " << "Point " << pV->position() << " Intersected with " << pColResult->intersectedTets.size() << "Points!"
                //    << "Closest point is on: " << pColResult->closestPoints[0] << "\n";
            }

            delete pColResult;
        }
    }
    clock_t timeConsumption_single = clock() - timeStart_single;

    printf("Time consumption by single thread: %d\n", timeConsumption_single);


    clock_t timeStart = clock();

    // parallelized
    tbb::parallel_for(0, (int)tetM2.vertices().size(), [&](int i)
        { 
            if (tetM2.vertices().hasBeenDeleted(i)) {
                return;
            }

            for (size_t k = 0; k < timeTries; k++)
            {
                TM::VPtr pV = &tetM2.vertices()[i];
                CollisionResult* pColResult = new CollisionResult;
                //pointQuery.vertexCollisionDetection(pV, &tetM2, pColResult);
                pointQuery.vertexCollisionDetectionAndClosestPointQuery(pV, &tetM2, pColResult);

                if (pColResult->intersectedTets.size() != 0)
                {
                    assert(pColResult->intersectedTets.size() == 1);
                    assert(pColResult->intersectedTMeshes[0] == &tetM1);

                    //std::cout << "Intersection found! " << "Point " << pV->position() << " Intersected with " << pColResult->intersectedTets.size() << "Points!"
                    //    << "Closest point is on: " << pColResult->closestPoints[0] << "\n";
                }

                delete pColResult;
            }

        }
    );

    clock_t timeConsumption = clock() - timeStart;

    printf("Time consumption by parallel: %d\n", timeConsumption);
   
   


}
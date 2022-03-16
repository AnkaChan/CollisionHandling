#include <limits>
#include <iostream>
#include <stdlib.h>
#include <ctime>

#include "PointQuery/PointQuery.h"
#include "PathFinder/TetMeshPathFinder.h"
#include <random>
//#define TBB_PARALLEL
#ifdef TBB_PARALLEL
#include "oneapi/tbb.h"

#endif // TBB_PARALLEL
#include <array>

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

    std::string outputFolder = "C:/Projects/Embree/EmbreeProjs/P07_TetrahedraQueryParallelDynamic/output";
    TM tetM1;
    tetM1._load_t("C:/Code/02_Graphics/CollisionHandling/Cpp_CollisionDetection/Data/bun_10KV20KF.t", true);

    TM tetM2;
    tetM2._load_t("C:/Code/02_Graphics/CollisionHandling/Cpp_CollisionDetection/Data/bun_10KV20KF.t", true);

    printf("Number of tetrahedra: %d\n", tetM2.tets().size());

    std::vector<TM::Ptr> tmeshes = { &tetM1, &tetM2 };
    PointQuery pointQuery(tmeshes);

    const size_t timeTries = 1;
    CPoint shift(0.01, 0., 0.);
    CPoint shiftStep(0.001, 0., 0.);
    int steps = 100;


    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist(0, tetM2.vertices().size()); // distribution in range [1, 6]

    std::vector<std::vector<PathFinder::TM::TPtr>> traverses;

    for (size_t iStep = 0; iStep < steps; iStep++)
    {
        clock_t timeStartUpdate = clock();

        shiftTetMesh(tetM1, shiftStep);

        std::vector<int> updatedTMIds = {0, 1 };
        pointQuery.update(&updatedTMIds);
        int numCollisions = 0;
        clock_t timeConsumptionUpdate = clock() - timeStartUpdate;


        clock_t timeStart = clock();

        int picked = dist(rng); 

        // parallelized
#ifdef TBB_PARALLEL
        tbb::parallel_for(0, (int)tetM2.vertices().size(), [&](int i)
#else
        for (int i = 0; i < (int)tetM2.vertices().size(); ++i)
#endif
            { 
                if (tetM2.vertices().hasBeenDeleted(i)) {
                    // return; 
                    break;
                }

                for (size_t k = 0; k < timeTries; k++)
                {
                    std::vector<std::vector<TM::TPtr>> traversedTets;
                    TM::VPtr pV = &tetM2.vertices()[i];


                    CollisionResult* pColResult = new CollisionResult;
                    if (i == picked)
                    {
                        traverses.clear();
                        pColResult->pTetTraversed = &traverses;
                    }
                    //pointQuery.vertexCollisionDetection(pV, &tetM2, pColResult);
                    pointQuery.vertexCollisionDetectionAndClosestPointQuery(pV, &tetM2, pColResult);

                    if (pColResult->intersectedTets.size() != 0)
                    {
                        assert(pColResult->intersectedTets.size() == 1);
                        assert(pColResult->intersectedTMeshes[0] == &tetM1);
                        numCollisions += 1;
                        //std::cout << "Intersection found! " << "Point " << pV->position() << " Intersected with " << pColResult->intersectedTets.size() << "Points!"
                        //    << "Closest point is on: " << pColResult->closestPoints[0] << "\n";

                        if (i == picked)
                        {
                            std::ostringstream aSs;
                            aSs << std::setfill('0') << std::setw(8) << iStep;
                            std::string outNumber = aSs.str();
                            std::string outName = outputFolder + "/" + "Traverse_" + outNumber + ".vtk";
                            tetM1._write_tet_list_to_vtk(outName.c_str(), (*pColResult->pTetTraversed)[0]);

                            std::string outNameSurface = outputFolder + "/" + "Surface_" + outNumber + ".ply";
                            pointQuery.pathFinders[0]->pM->write_ply(outNameSurface.c_str());

                            std::array<double, 3> closestP = { pColResult->closestPoints[0][0], pColResult->closestPoints[0][1], pColResult->closestPoints[0][2] };
                            std::array<double, 3> queryP = { pV->position()[0], pV->position()[1], pV->position()[2]};
                            std::vector<std::array<double, 3>> pts = {
                               closestP,
                               queryP
                            };

                            std::vector<std::array<int, 3>> fs;

                            M m;
                            std::string outNameQueryResult = outputFolder + "/" + "QueryResult_" + outNumber + ".ply";
                            m.readVFList(&pts, &fs,  nullptr, false);
                            m.write_ply(outNameQueryResult.c_str());
                        }
                    }


                    delete pColResult;
                }
            }
#ifdef TBB_PARALLEL
        );
#endif
        clock_t timeConsumption = clock() - timeStart;
        printf("Time consumption by parallel: %d, by updating: %d, number of collided verts: %d\n", timeConsumption, timeConsumptionUpdate, numCollisions);

    }


   


}
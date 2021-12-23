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
    
int main()
{

    // testPathFinder();
    TM pTetM;
    pTetM._load_t("C:/Code/02_Graphics/CollisionHandling/Cpp_CollisionDetection/Data/bun_10KV20KF.t", true);
    pTetM._write_vtk("test.vtk");

    TM::TPtr pT = pTetM.idTet(100);

    std::vector<TM::TPtr> tetsToSave ;
    for (TM::HFPtr pHF : TIt::T_HFIterator(pT)) {
        TM::TPtr pNeiT = TM::HalfFaceTet(pHF->dual());
        if (pNeiT != NULL) {
            tetsToSave.push_back(pNeiT);
        }

    }

    pTetM._write_tet_list_to_vtk("test_TTIter.vtk", tetsToSave);

    tetsToSave.clear();

    TM::VPtr pV = pTetM.idVertex(100);
    for (TM::TVPtr pNeiTV : TIt::V_TVIterator(pV)) {
        if (pNeiTV != NULL) {
            tetsToSave.push_back(TM::TVertexTet(pNeiTV));
        }

    }
    pTetM._write_tet_list_to_vtk("test_VTIter.vtk", tetsToSave);

    CPoint rayOrigin(0., 0.1, 0.);
    CPoint rayVector(0., 1., 0.);
    TM::TPtr pTEmbrace = nullptr;

    TetMeshPathFinder pathFinder(&pTetM);

    for (TM::TPtr pT : TIt::TM_TIterator(&pTetM)) {
        CPoint center = TM::TetCentroid(pT);

        for (TM::HFPtr pHF : TIt::T_HFIterator(pT)) {
            CPoint halfFacePts[3];
            TM::HalfFace3Points(pHF, halfFacePts);
            CPoint normalD = TM::HalfFaceOrientedArea(pHF);
            // printf("%e\n", (center- halfFacePts[0]) * normalD);

            assert((center - halfFacePts[0]) * normalD < 0);
        }

        if (TM::PointInTet(pT, rayOrigin))
        {
            pTEmbrace = pT;
            printf("Ray origin in Tet with Id: %d\n", pT->id());
            break;
        }
    }

    std::vector<TM::TPtr> traversedTVec;

    /*TM::HFPtr pSurfaceHF = pathFinder.rayTMeshTraverse(pTEmbrace, rayOrigin, rayVector, &traversedTVec);

    printf("Traversed Tets:\n");
    for (TM::TPtr pTTraversed : traversedTVec) {
        printf("%d->", pTTraversed->id());

    }
    printf("surface HF: %d.\n", pSurfaceHF->index());
    pTetM._write_tet_list_to_vtk("test_traversed.vtk", traversedTVec);*/

    std::vector<TM::Ptr> tmeshes = { &pTetM };
    PointQuery pointQuery(tmeshes);

    //std::random_device rd;
    //std::default_random_engine eng(rd());
    //std::uniform_real_distribution<double> distr(1e-3, 1.0-1e-3);

    srand(12345);

    int tryPerTet = 100;

    for (TM::TPtr pT : TIt::TM_TIterator(&pTetM))
    {
        printf("Testing Tet: %d\n", pT->id());
        for (size_t i = 0; i < tryPerTet; i++)
        {
            rayOrigin = CPoint(0., 0., 0.);

            double sum = 0.;
            for (TM::VPtr pV : TIt::T_VIterator(pT))
            {
                double weight = rand() / double(RAND_MAX);
                sum += weight;
                rayOrigin += weight * pV->position();
            }

            rayOrigin = rayOrigin / sum;
            TM::PointInTet(pT, rayOrigin);

            pointQuery.queryPoint(rayOrigin, pT, 0, inf, false, false);
            //std::cout << "Without Feasible Region Check: Closest point: " << pointQuery.result.closestP << " Distance: " << pointQuery.result.d
            //    << " Face traversed: " << pointQuery.result.nFaceTraversed << std::endl;
            CPoint closestP1 = pointQuery.result.closestP;

            pointQuery.queryPoint(rayOrigin, pT, 0, inf, true, false);
            //std::cout << "With Feasible Region Check: Closest point: " << pointQuery.result.closestP << " Distance: " << pointQuery.result.d
            //    << " Face traversed: " << pointQuery.result.nFaceTraversed << std::endl;
            CPoint closestP2 = pointQuery.result.closestP;

            traversedTVec.clear();
            pointQuery.queryPoint(rayOrigin, pT, 0, inf, true, true, &traversedTVec);
            //std::cout << "With Feasible Region Check and traverse Check: Closest point: " << pointQuery.result.closestP << " Distance: " << pointQuery.result.d
            //    << " Face traversed: " << pointQuery.result.nFaceTraversed << std::endl;
            CPoint closestP3 = pointQuery.result.closestP;
            
            double diff12 = (closestP1 - closestP2).norm();
            double diff13 = (closestP1 - closestP3).norm();

            if (diff12 > 1e-5) {
                printf("Diff1 larger than threshold: %f", diff12);
                assert(false);
            }
            if (diff13 > 1e-5) {
                printf("Diff2 larger than threshold: %f", diff13);
                pTetM._write_tet_list_to_vtk("test_traversed.vtk", traversedTVec);

                assert(false);
            }

        }
       
    }




}
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

    //double weightsDebug[4] = { 0.82036805322428052, 0.0021973326822717978 , 0.094363231299783315 , 0.74208807641834773 };
    ////double weightsDebug[4] = {0.82036805322428052, 0.021973326822717978 , 0.094363231299783315 , 0.74208807641834773};
    //int debugTId = 15359;
    //pT = (TM::TPtr)pTetM.idTet(debugTId);

    //double sum = 0.;
    //int i = 0;
    //rayOrigin = CPoint(0., 0., 0.);
    //for (TM::VPtr pV : TIt::T_VIterator(pT))
    //{
    //    double weight = weightsDebug[i];
    //    i++;
    //    if (weight <= 1e-5)
    //    {
    //        weight = 1e-5;
    //    }
    //    sum += weight;
    //    rayOrigin += weight * pV->position();
    //}
    //printf("Edge length for Tet %d:\n", debugTId);
    //for (TM::EPtr pE : TIt::T_EIterator(pT))
    //{
    //    double eL = TM::EdgeLength(pE);
    //    printf("%f\n", eL);
    //}
    //rayOrigin = rayOrigin / sum;
    //CPoint tetCentroid = TM::TetCentroid(pT);
    //rayOrigin = rayOrigin + (tetCentroid - rayOrigin) * 0.01;

    //assert(TM::PointInTet(pT, rayOrigin));
    //traversedTVec.clear();
    //pointQuery.queryPoint(rayOrigin, pT, 0, inf, true, true, &traversedTVec);

    srand(12345);
    int tryPerTet = 10000;

    clock_t timeStart = clock();

    for (TM::TPtr pT : TIt::TM_TIterator(&pTetM))
    {
        if (!(pT->id() % 100)) {
            clock_t timeConsumption = clock() - timeStart;
            printf("Testing Tet: %d, time consumption: %d ms, %f per 100 tet.\n", pT->id(), timeConsumption, timeConsumption * 100. / pT->id());
        }
        for (size_t i = 0; i < tryPerTet; i++)
        {
            rayOrigin = CPoint(0., 0., 0.);

            double sum = 0.;
            std::vector<double> weights;
            for (TM::VPtr pV : TIt::T_VIterator(pT))
            {
                double weight = rand() / double(RAND_MAX);

                //if (weight <= 1e-5)
                //{
                //    weight = 1e-5;
                //}
                weights.push_back(weight);
                sum += weight;
                rayOrigin += weight * pV->position();
            }

            rayOrigin = rayOrigin / sum;
            assert(TM::PointInTet(pT, rayOrigin));

            CPoint tetCentroid = TM::TetCentroid(pT);
            rayOrigin = rayOrigin + (tetCentroid - rayOrigin) * 0.01;

            pointQuery.queryPoint(rayOrigin, pT, 0, inf, false, false);
            //std::cout << "Without Feasible Region Check: Closest point: " << pointQuery.result.closestP << " Distance: " << pointQuery.result.d
            //    << " Face traversed: " << pointQuery.result.nFaceTraversed << std::endl;
            CPoint closestP1 = pointQuery.result.closestP;

            //pointQuery.queryPoint(rayOrigin, pT, 0, inf, true, false);
            ////std::cout << "With Feasible Region Check: Closest point: " << pointQuery.result.closestP << " Distance: " << pointQuery.result.d
            ////    << " Face traversed: " << pointQuery.result.nFaceTraversed << std::endl;
            //CPoint closestP2 = pointQuery.result.closestP;

            traversedTVec.clear();
            //pointQuery.queryPoint(rayOrigin, pT, 0, inf, true, true, &traversedTVec);
            pointQuery.queryPoint(rayOrigin, pT, 0, inf, true, true, nullptr);
            //std::cout << "With Feasible Region Check and traverse Check: Closest point: " << pointQuery.result.closestP << " Distance: " << pointQuery.result.d
            //    << " Face traversed: " << pointQuery.result.nFaceTraversed << std::endl;
            CPoint closestP3 = pointQuery.result.closestP;
            
            //double diff12 = (closestP1 - closestP2).norm();
            double diff13 = (closestP1 - closestP3).norm();

            //if (diff12 > 1e-6) {
            //    printf("In Tet: %d\n", pT->id());
            //    printf("Diff1 larger than threshold: %f\n", diff12);
            //    assert(false);
            //}
            if (diff13 > 1e-6) {
                printf("In Tet: %d\n", pT->id());
                printf("Diff2 larger than threshold: %f\n", diff13);
                pTetM._write_tet_list_to_vtk("test_traversed.vtk\n", traversedTVec);

                assert(false);
            }

        }
       
    }




}
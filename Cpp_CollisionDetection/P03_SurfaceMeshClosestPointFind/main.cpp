#include <limits>
#include <iostream>

#include <ctime>

#include "PointQuery/PointQuery.h"
#include "PathFinder/TetMeshPathFinder.h"

using namespace  embree;
using namespace  PathFinder;

M::Ptr m;

void testPathFinder() {
    m = new M;
    m->read_obj("C:/Code/02_Graphics/CollisionHandling/Cpp_CollisionDetection/Data/TestMesh2_2TrianglesPlane.obj");

    TetMeshPathFinder pathFinder(m, nullptr);
    CPoint P1(-0.5, 0.5, -0.5);

    for (M::FPtr pF : It::MFIterator(m))
    {
        bool inFeasibleRegion = pathFinder.checkFaceFeasibleRegion(pF, P1);
        printf("P1 in Face %d: %d\n", pF->id(), inFeasibleRegion);

    }

    CPoint P2(0.5, 0.5, -0.5);

    for (M::FPtr pF : It::MFIterator(m))
    {
        bool inFeasibleRegion = pathFinder.checkFaceFeasibleRegion(pF, P2);
        printf("P2 in Face %d: %d\n", pF->id(), inFeasibleRegion);

    }

    CPoint P3(0.5, -0.5, -0.5);

    for (M::FPtr pF : It::MFIterator(m))
    {
        bool inFeasibleRegion = pathFinder.checkFaceFeasibleRegion(pF, P3);
        printf("P3 in Face %d: %d\n", pF->id(), inFeasibleRegion);

    }

    delete m;
    m = new M;

    m->read_obj("C:/Code/02_Graphics/CollisionHandling/Cpp_CollisionDetection/Data/TestMesh3_BendLargerThanPi.obj");
    CPoint E_P1(0., 0., 0.5);
    CPoint E_P2(1.01, 1.01, 0.3);
    CPoint E_P3(0.99, 0.99, 0.3);
    CPoint E_P4(0.1, 0., 0.009);


    for (M::EPtr pE : It::MEIterator(m))
    {
        M::FPtr pF1 = m->edgeFace1(pE);
        M::FPtr pF2 = m->edgeFace2(pE);

        if (pF1 != nullptr && pF2 != nullptr) {
            bool inFeasibleRegion = pathFinder.checkEdgeFeasibleRegion(pE, E_P1);
            printf("E_P1 in Edge %d: %d\n", pE->id(), inFeasibleRegion);

            inFeasibleRegion = pathFinder.checkEdgeFeasibleRegion(pE, E_P2);
            printf("E_P2 in Edge %d: %d\n", pE->id(), inFeasibleRegion);


            inFeasibleRegion = pathFinder.checkEdgeFeasibleRegion(pE, E_P3);
            printf("E_P3 in Edge %d: %d\n", pE->id(), inFeasibleRegion);

            inFeasibleRegion = pathFinder.checkEdgeFeasibleRegion(pE, E_P4);
            printf("E_P4 in Edge %d: %d\n", pE->id(), inFeasibleRegion);
        }

    }


    delete m;
    m = new M;

    m->read_obj("C:/Code/02_Graphics/CollisionHandling/Cpp_CollisionDetection/Data/TestMesh4_ConverxPoint.obj");
    CPoint V_P1(0., 0., 0.5);
    CPoint V_P2(1., 1., 0.5);
    CPoint V_P3(0.1, 0.1, 1.0);

    for (M::VPtr pV : It::MVIterator(m))
    {
        if (pV->id() == 4) {
            bool inFeasibleRegion = pathFinder.checkVertexFeasibleRegion(pV, V_P1);
            printf("P1 in Vertex %d: %d\n", pV->id(), inFeasibleRegion);

            inFeasibleRegion = pathFinder.checkVertexFeasibleRegion(pV, V_P2);
            printf("P2 in Vertex %d: %d\n", pV->id(), inFeasibleRegion);

            inFeasibleRegion = pathFinder.checkVertexFeasibleRegion(pV, V_P3);
            printf("P3 in Vertex %d: %d\n", pV->id(), inFeasibleRegion);

            printf("Raise V up.\n");
            pV->point()[2] = 0.1;

            inFeasibleRegion = pathFinder.checkVertexFeasibleRegion(pV, V_P1);
            printf("P1 in Vertex %d: %d\n", pV->id(), inFeasibleRegion);

            inFeasibleRegion = pathFinder.checkVertexFeasibleRegion(pV, V_P2);
            printf("P2 in Vertex %d: %d\n", pV->id(), inFeasibleRegion);

            inFeasibleRegion = pathFinder.checkVertexFeasibleRegion(pV, V_P3);
            printf("P3 in Vertex %d: %d\n", pV->id(), inFeasibleRegion);
        }
    }

    delete m;

}

int main()
{

    // testPathFinder();
    m = new M;

    // m->read_obj("C:/Code/02_Graphics/CollisionHandling/Cpp_CollisionDetection/Data/human.obj");
    m->read_obj("C:/Code/02_Graphics/CollisionHandling/Cpp_CollisionDetection/Data/bun_zipper.obj");
    m->reinitializeFId();
    m->reinitializeVId();

    for (M::EPtr pE : It::MEIterator(m)) {
        M::FPtr pF1 = m->edgeFace1(pE);
        M::FPtr pF2 = m->edgeFace2(pE);

        assert(pF1 != nullptr && pF2 != nullptr);
    }

    std::vector<M::Ptr> meshes;
    meshes.push_back(m);

    PointQuery pointQuery(meshes);
    CPoint p(0.10, 0.1, 0.11);
    pointQuery.queryPoint(p, 0, inf, false);
    std::cout << "Without Feasible Region Check: Closest point: " << m->vertices()[pointQuery.result.primID].point() << " Distance: " << pointQuery.result.d 
        << " Face traversed: " << pointQuery.result.nFaceTraversed << std::endl;

    pointQuery.queryPoint(p, 0, inf, true);
    std::cout << "With Feasible Region Check: Closest point: " << pointQuery.result.closestP << " Distance: " << pointQuery.result.d
        << " Face traversed: " << pointQuery.result.nFaceTraversed << std::endl;

}
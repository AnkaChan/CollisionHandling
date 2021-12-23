#include <embree3/rtcore.h>
#include <limits>
#include <iostream>

#include "common/math/vec2.h"
#include "common/math/vec3.h"
#include "common/math/vec4.h"
#include "common/math/affinespace.h"
#include <MeshFrame/core/Mesh/Iterators2.h>
#include <MeshFrame/core/Mesh/MeshCoreHeaders.h>

#include <ctime>

typedef MeshLib::CBaseMesh<MeshLib::CVertex, MeshLib::CEdge, MeshLib::CFace, MeshLib::CHalfEdge> CMesh;

CMesh* m;

using namespace  embree;
typedef MeshLib::CIterators<CMesh> It;

struct ClosestPointResult
{
    ClosestPointResult()
        : primID(RTC_INVALID_GEOMETRY_ID)
        , geomID(RTC_INVALID_GEOMETRY_ID)
    {}

    Vec3f p;
    unsigned int primID;
    unsigned int geomID;
};

/* vertex and triangle layout */
struct Vertex { float x, y, z, r; }; // FIXME: rename to Vertex4f
struct Triangle { int v0, v1, v2; };

float* vb = nullptr;
int nFaceTraversed = 0;

struct TriangleMesh
{
    ALIGNED_STRUCT_(16)
    Vertex* vertices;
    Triangle* triangles;
    unsigned int num_vertices;
    unsigned int num_triangles;

    TriangleMesh()
        : vertices(nullptr), triangles(nullptr) {}

    ~TriangleMesh() {
        if (vertices) alignedFree(vertices);
        if (triangles) alignedFree(triangles);
    }

private:
    TriangleMesh(const TriangleMesh& other) DELETED; // do not implement
    TriangleMesh& operator= (const TriangleMesh& other) DELETED; // do not implement
};

TriangleMesh* g_triangle_meshes[4] = {
nullptr, nullptr, nullptr, nullptr
};

/* Types of buffers */
enum ClosestPointOnTriangleType
{
    AtA,
    AtB,
    AtC,
    AtAB,
    AtBC,
    AtAC,
    AtInterior
};

Vec3fa closestPointTriangle(Vec3fa const& p, Vec3fa const& a, Vec3fa const& b, Vec3fa const& c, ClosestPointOnTriangleType &pointType)
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


bool closestPointFunc(RTCPointQueryFunctionArguments* args)
{
    nFaceTraversed++;
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

    /*
     * Get triangle information in local space
     */

    CMesh::FPtr pF = &(m->getFContainer()[primID]);

    Vec3fa vs[3];

    int iFV = 0;
    for (CMesh::VPtr pVF : It::FVIterator(pF))
    {
        vs[iFV].x = pVF->point()[0];
        vs[iFV].y = pVF->point()[1];
        vs[iFV].z = pVF->point()[2];
        iFV += 1;
    }

    /*
     * Bring query and primitive data in the same space if necessary.
     */
    if (stackSize > 0 && args->similarityScale > 0)
    {
        // Instance transform is a similarity transform, therefore we 
        // can comute distance insformation in instance space. Therefore,
        // transform query position into local instance space.
        AffineSpace3fa const& m = (*(AffineSpace3fa*)context->world2inst[stackPtr]);
        q = xfmPoint(m, q);
    }
    else if (stackSize > 0)
    {
        // Instance transform is not a similarity tranform. We have to transform the
        // primitive data into world space and perform distance computations in
        // world space to ensure correctness.
        vs[0] = xfmPoint(inst2world, vs[0]);
        vs[1] = xfmPoint(inst2world, vs[1]);
        vs[2] = xfmPoint(inst2world, vs[2]);
    }
    else {
        // Primitive is not instanced, therefore point query and primitive are
        // already in the same space.
    }

    /*
     * Determine distance to closest point on triangle (implemented in
     * common/math/closest_point.h), and transform in world space if necessary.
     */
    ClosestPointOnTriangleType pointType;
    const Vec3fa p = closestPointTriangle(q, vs[0], vs[1], vs[2], pointType);
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
        // args->query->radius = d;
        ClosestPointResult* result = (ClosestPointResult*)args->userPtr;
        result->p = args->similarityScale > 0 ? xfmPoint(inst2world, p) : p;
        result->primID = primID;
        result->geomID = geomID;
        return true; // Return true to indicate that the query radius changed.
    }


    return false;
}

int main()
{
    m = new CMesh;
    //m->read_obj("C:/Code/01_Vision/DenseLocalizationTexture/S02_TextureDesign/GridMesh_200x200.obj");
    m->read_obj("C:/Code/02_Graphics/CollisionHandling/Cpp_CollisionDetection/Data/bun_zipper.obj");


    RTCDevice device = rtcNewDevice(NULL);
    RTCScene scene = rtcNewScene(device);
    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

    size_t numVerts = m->numVertices();
    size_t numFaces = m->numFaces();

    printf_s("Number of Verts: %d Number of Faces: %d\n", numVerts, numFaces);

    vb = (float*)rtcSetNewGeometryBuffer(geom,
        RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float), numVerts);

    for (CMesh::VPtr pV : It::MVIterator(m))
    {
        vb[pV->id() * 3] = pV->point()[0];
        vb[pV->id() * 3+1] = pV->point()[1];
        vb[pV->id() * 3+2] = pV->point()[2];
    }

    unsigned* ib = (unsigned*)rtcSetNewGeometryBuffer(geom,
        RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned), numFaces);

    for (CMesh::FPtr pF : It::MFIterator(m))
    {
        int iFV = 0;

        for (CMesh::VPtr pVF : It::FVIterator(pF))
        {
            ib[pF->id() * 3 + iFV] = pVF->id();
            iFV += 1;
        }
    }

    rtcSetGeometryPointQueryFunction(geom, closestPointFunc);


    rtcCommitGeometry(geom);
    rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);
    rtcCommitScene(scene);

    RTCPointQuery query;
    query.x = 0.5f;
    query.y = 0.5f;
    query.z = 0.5f;
    query.radius = 0.0;
    query.time = 0.f;

    ClosestPointResult result;
    RTCPointQueryContext context;
    rtcInitPointQueryContext(&context);
    std::clock_t timeStart = std::clock();
    rtcPointQuery(scene, &query, &context, nullptr, (void*)&result);
    float processTime = float(std::clock()) - float(timeStart);
    printf_s("Traverse %d face time consumption: %f\n", nFaceTraversed, processTime);

    assert(result.primID != RTC_INVALID_GEOMETRY_ID || result.geomID != RTC_INVALID_GEOMETRY_ID);

    rtcReleaseScene(scene);
    rtcReleaseDevice(device);
}
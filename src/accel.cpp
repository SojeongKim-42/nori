/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/accel.h>
#include <nori/node.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

void Accel::addMesh(Mesh *mesh) {
    if (m_mesh) throw NoriException("Accel: only a single mesh is supported!");
    m_mesh = mesh;
    m_bbox = m_mesh->getBoundingBox();
}

std::vector<uint32_t> Accel::build(const Ray3f &ray, OctreeNode *node_,
                                   std::vector<uint32_t> triangles, int depth) const {
    OctreeNode *node = node_;
    std::vector<uint32_t> itsTriangles(0); //만나는 삼각형

    if (triangles.size() == 0 ||triangles.size() <= 10||depth>9) {
        return triangles;
    }

    node->buildChildOctree();
    std::vector<std::vector<uint32_t>> subTriangles(8);

    for (int i = 0; i < 8; ++i) {
        OctreeNode *childNode = node->children[i];
        if (childNode !=nullptr && childNode->box.rayIntersect(ray)) {  // childNode가 ray와 만난다면 자식 노드 다시 생성
            for (uint32_t idx : triangles) { // 모든 삼각형이 자식 노드의 박스와 만나는지 확인 후 subtriangle에 push
                if (childNode->box.overlaps(m_mesh->getBoundingBox(idx))) {  // 삼각형의 바운딩박스와 박스가 겹쳐지면 만남
                    subTriangles[i].push_back(idx);
                }
            }
            std::vector<uint32_t> result =
                build(ray, childNode, subTriangles[i], depth+1);
            itsTriangles.insert(itsTriangles.end(), result.begin(), result.end());
        }
        delete childNode;
    }
    return itsTriangles;
}

bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its,
                         bool shadowRay) const {
    bool foundIntersection = false;  // Was an intersection found so far?
    uint32_t f = (uint32_t)-1;  // Triangle index of the closest intersection

    Ray3f ray(ray_);  /// Make a copy of the ray (we will need to update its
                      /// '.maxt' value)

    OctreeNode *node = new OctreeNode(m_bbox);  // 최초의 node
    std::vector<uint32_t> allTriangles(0);
    for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx) {
        allTriangles.push_back(idx);
    }

    std::vector<uint32_t> triangles(0);
    int depth=0;
    triangles = build(ray, node, allTriangles, depth);  // ray가 만나는 triangle들

    for (uint32_t idx : triangles) {
        float u, v, t;
        if (m_mesh->rayIntersect(idx, ray, u, v, t)) {
            /* An intersection was found! Can terminate
               immediately if this is a shadow ray query */
            if (shadowRay) return true;
            ray.maxt = its.t = t;
            its.uv = Point2f(u, v);
            its.mesh = m_mesh;
            f = idx;
            foundIntersection = true;
        }
    }

    if (foundIntersection) {
        /* At this point, we now know that there is an intersection,
           and we know the triangle index of the closest such intersection.

           The following computes a number of additional properties which
           characterize the intersection (normals, texture coordinates, etc..)
        */

        /* Find the barycentric coordinates */
        Vector3f bary;
        bary << 1 - its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh *mesh = its.mesh;
        const MatrixXf &V = mesh->getVertexPositions();
        const MatrixXf &N = mesh->getVertexNormals();
        const MatrixXf &UV = mesh->getVertexTexCoords();
        const MatrixXu &F = mesh->getIndices();

        /* Vertex indices of the triangle */
        uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        /* Compute the intersection positon accurately
           using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) + bary.y() * UV.col(idx1) +
                     bary.z() * UV.col(idx2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1 - p0).cross(p2 - p0).normalized());

        if (N.size() > 0) {
            /* Compute the shading frame. Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

            its.shFrame =
                Frame((bary.x() * N.col(idx0) + bary.y() * N.col(idx1) +
                       bary.z() * N.col(idx2))
                          .normalized());
        } else {
            its.shFrame = its.geoFrame;
        }
    }

    return foundIntersection;
}

NORI_NAMESPACE_END

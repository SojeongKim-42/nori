#pragma once

#include <nori/accel.h>
#include <nori/bbox.h>

NORI_NAMESPACE_BEGIN

class OctreeNode {
   public:
    const int MAX_DEPTH = 9;
    BoundingBox3f m_box;  // node가 가리키는 box
    bool m_leaf;
    int m_depth;
    std::vector<uint32_t> m_triangles;
    std::array<OctreeNode *, 8> m_children = {nullptr};  // node의 children

    OctreeNode(BoundingBox3f bbox, std::vector<uint32_t> triangles) {
        m_box = bbox;
        m_triangles = triangles;
        m_leaf = false;
        m_depth = 0;
    }

    OctreeNode(BoundingBox3f bbox, std::vector<uint32_t> triangles, int depth) {
        m_box = bbox;
        m_triangles = triangles;
        m_leaf = false;
        m_depth = depth;
    }

    void buildChildOctree(Mesh *mesh) {
        // 노드의 bounding box를 8개로 나눕니다.
        Vector3f min = m_box.min;
        Vector3f max = m_box.max;
        Vector3f mid = (min + max) * 0.5f;
        float sideLengthX = mid.x() - min.x();
        float sideLengthY = mid.y() - min.y();
        float sideLengthZ = mid.z() - min.z();

        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                for (int k = 0; k < 2; ++k) {
                    Point3f childMin(min.x() + i * (sideLengthX),
                                     min.y() + j * (sideLengthY),
                                     min.z() + k * (sideLengthZ));
                    Point3f childMax(childMin.x() + (sideLengthX),
                                     childMin.y() + (sideLengthY),
                                     childMin.z() + (sideLengthZ));
                    BoundingBox3f childBox(childMin, childMax);

                    std::vector<uint32_t> childTriangles;
                    for (uint32_t idx : m_triangles) {
                        if (childBox.overlaps(mesh->getBoundingBox(idx))) {
                            childTriangles.push_back(idx);
                        }
                    }

                    if (childTriangles.size() == 0) {
                        m_children[int(i * 4 + j * 2 + k)] = nullptr;
                    } else {
                        m_children[int(i * 4 + j * 2 + k)] = new OctreeNode(
                            childBox, childTriangles, m_depth + 1);
                    }
                }
            }
        }
    }

    void build(Mesh *mesh) {
        if (m_triangles.size() <= 10 || m_depth == MAX_DEPTH) {
            m_leaf = true;
            std::fill(m_children.begin(), m_children.end(), nullptr);
            return;
        }

        buildChildOctree(mesh);
        for (OctreeNode *child : m_children) {
            if (child != nullptr) {
                child->build(mesh);
            }
        }
        return;
    }

    std::vector<OctreeNode *> sortIntersectChildNode(Ray3f ray) {
        std::vector<OctreeNode *> intersectChilds;
        for (OctreeNode *child : m_children) {
            if (child != nullptr && child->m_box.rayIntersect(ray)) {
                intersectChilds.push_back(child);
            }
        }

        std::sort(intersectChilds.begin(), intersectChilds.end(),
                  [&](OctreeNode *a, OctreeNode *b) {
                      float farA, farB, distanceA, distanceB;
                      a->m_box.rayIntersect(ray, distanceA, farA);
                      b->m_box.rayIntersect(ray, distanceB, farB);
                      return distanceA < distanceB;
                  });

        return intersectChilds;
    }

    uint32_t findIntersectionIdx(Ray3f &ray, Mesh *mesh, Intersection &its) {
        uint32_t closest = (uint32_t)-1;
        if (m_leaf) {
            for (uint32_t idx : m_triangles) {
                float u, v, t;
                if (mesh->rayIntersect(idx, ray, u, v, t)) {
                    /* At this point, we now know that there is an intersection,
                    and we know the triangle index of the closest such
                    intersection.

                    The following computes a number of additional properties
                    which characterize the intersection (normals, texture
                    coordinates, etc..)
                    */
                    ray.maxt = t;
                    its.uv = Point2f(u, v);
                    its.mesh = mesh;
                    closest = idx;
                }
            }
            return closest;  // 삼각형과 만나지 못할 때 -1 리턴
        }

        std::vector<OctreeNode *> intersectChilds = sortIntersectChildNode(ray);
        for (OctreeNode *child : intersectChilds) {
            uint32_t result = child->findIntersectionIdx(ray, mesh, its);
            if (result != (uint32_t)-1) closest = result;
        }
        return closest;
    }
};

NORI_NAMESPACE_END

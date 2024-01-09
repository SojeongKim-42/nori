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
    Point3f m_origin;
    std::vector<uint32_t> m_triangles;
    std::array<OctreeNode *, 8> m_children = {nullptr};  // node의 children

    OctreeNode(BoundingBox3f bbox, std::vector<uint32_t> triangles) {
        m_box = bbox;
        m_triangles = triangles;
        m_leaf = false;
        m_depth = 0;
    }

    OctreeNode(BoundingBox3f bbox, std::vector<uint32_t> triangles, int depth,
               Point3f origin) {
        m_box = bbox;
        m_triangles = triangles;
        m_leaf = false;
        m_depth = depth;
        m_origin = origin;
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
                    Vector3f childMin(min.x() + i * (sideLengthX),
                                      min.y() + j * (sideLengthY),
                                      min.z() + k * (sideLengthZ));
                    Vector3f childMax(childMin.x() + (sideLengthX),
                                      childMin.y() + (sideLengthY),
                                      childMin.z() + (sideLengthZ));
                    BoundingBox3f childBox(childMin, childMax);

                    std::vector<uint32_t> childTriangles(0);
                    for (uint32_t idx : m_triangles) {
                        if (childBox.overlaps(mesh->getBoundingBox(idx))) {
                            childTriangles.push_back(idx);
                        }
                    }

                    m_children[int(i * 4 + j * 2 + k)] = new OctreeNode(
                        childBox, childTriangles, m_depth + 1, m_origin);
                }
            }
        }
    }

    void build(Mesh *mesh) {
        if (m_triangles.size() == 0 || m_triangles.size() <= 10 ||
            m_depth == MAX_DEPTH) {
            m_leaf = true;
            for (int i4 = 0; i4 < 8; ++i4) {
                m_children[i4] = nullptr;
            }
            return;
        }

        buildChildOctree(mesh);
        for (int i5 = 0; i5 < 8; ++i5) {
            m_children[i5]->build(mesh);
        }
        return;
    }

    void sortNode() {
        if (m_children[0] != nullptr) {
            std::sort(m_children.begin(), m_children.end(),
                      [&](OctreeNode *a, OctreeNode *b) {
                          float distanceA = a->m_box.distanceTo(m_origin);
                          float distanceB = b->m_box.distanceTo(m_origin);
                          return distanceA < distanceB;
                      });

            for (int i = 0; i < 8; ++i) {
                m_children[i]->sortNode();
            }
        }
    }

    std::vector<uint32_t> findTriangles(Ray3f ray,
                                        float minD) {
        std::vector<uint32_t> answerTriangles;
        // 정렬 ---
        if (ray.o != m_origin) { 
            m_origin = ray.o;
            sortNode();
        }

        // 교차하는 삼각형 반환 ---
        if (m_leaf) {
            return m_triangles;
        }

        for (int i0 = 0; i0 < 8; ++i0) {
            if (m_children[i0]->m_box.rayIntersect(ray) &&
                minD >= m_children[i0]->m_box.distanceTo(m_origin)) {
                std::vector<uint32_t> result =
                    m_children[i0]->findTriangles(ray, minD);
                if (true) { //ray와 bbox 안의 삼각형이 만남.
                    answerTriangles.insert(answerTriangles.end(), result.begin(),
                                   result.end());
                    // minD = m_children[i0]->m_box.distanceTo(m_origin);
                }
            }
        }
        return answerTriangles;
    }
};

NORI_NAMESPACE_END

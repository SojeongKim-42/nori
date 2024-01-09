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
    float m_distance;
    Point3f m_origin;
    std::vector<uint32_t> m_triangles;
    std::array<OctreeNode *, 8> m_children={nullptr};  // node의 children

    OctreeNode(BoundingBox3f bbox, std::vector<uint32_t> triangles) {
        m_box = bbox;
        m_triangles = triangles;
        m_leaf = false;
        m_depth = 0;
        m_distance = 0;
        m_origin = (0.0f, 0.0f, 0.0f);
    }

    OctreeNode(BoundingBox3f bbox, std::vector<uint32_t> triangles, int depth, Point3f origin) {
        m_box = bbox;
        m_triangles = triangles;
        m_leaf = false;
        m_depth = depth;
        m_distance = 0;
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
                    for (uint32_t idx :
                         m_triangles) {  // 모든 삼각형이 자식 노드의 박스와
                                         // 만나는지 확인 후 subtriangle에 push
                        if (childBox.overlaps(mesh->getBoundingBox(
                                idx))) {  // 삼각형의 바운딩박스와 박스가
                                          // 겹쳐지면 만남
                            childTriangles.push_back(idx);
                        }
                    }

                    m_children[int(i * 4 + j * 2 + k)] =
                        new OctreeNode(childBox, childTriangles, m_depth+1, m_origin);
                }
            }
        }
    }

    void build(Mesh *mesh) {
        if (m_triangles.size() == 0 || m_triangles.size() <= 10 || m_depth == MAX_DEPTH) {
            m_leaf = true;
            for (int i4 = 0; i4 < 8; ++i4) {
                m_children[i4]=nullptr;
            }
            return;
        }

        buildChildOctree(mesh);
        for (int i5 = 0; i5 < 8; ++i5) {
            m_children[i5]->build(mesh);
        }
        return;
    }

    void sortNode(Ray3f ray) {
        std::array<OctreeNode *, 8> sortedChildren = {nullptr};
        std::vector<float> sortedDistanceIndex;
        if (m_children[0] != nullptr) {
            OctreeNode *ptr=m_children[0];
            for (int i1 = 0; i1 < 8; ++i1) {
                m_children[i1]->m_distance = m_children[i1]->m_box.distanceTo(
                    m_origin);  // 모든 박스의 distance 계산
                sortedDistanceIndex.push_back(i1);
            }

            int idx = 0;
            for (int i2 : sortedDistanceIndex) {
                sortedChildren[idx] = m_children[i2];
                idx++;
            }
            m_children = sortedChildren;

            for (int i3 = 0; i3 < 8; ++i3) {
                    m_children[i3]->sortNode(ray);
            }
        }
    }

    std::vector<uint32_t> findTriangles(Ray3f ray,
                                        std::vector<uint32_t> answerTriangles) {
        // 정렬 ---
        if (ray.o != m_origin) {  // origin이 바뀌었을 때만 정렬
            m_origin = ray.o;
            sortNode(ray);
        }

        // answerTriangle반환 (교차하는 삼각형) ---
        if (m_leaf) {
            answerTriangles.insert(answerTriangles.end(), m_triangles.begin(),
                                   m_triangles.end());
            return answerTriangles;
        }

        for (int i0 = 0; i0 < 8; ++i0) {
            float nearT, farT;
            if (m_children[i0]->m_box.rayIntersect(ray, nearT, farT)) {
                if(ray.maxt> farT){
                    ray.maxt=farT;
                    ray.mint=nearT;
                    answerTriangles= m_children[i0]->findTriangles(ray,
                    answerTriangles);
                }
                answerTriangles =
                    m_children[i0]->findTriangles(ray, answerTriangles);
            }
        }
        return answerTriangles;
    }
};

NORI_NAMESPACE_END

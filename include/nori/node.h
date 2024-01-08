#pragma once

#include <nori/accel.h>

NORI_NAMESPACE_BEGIN

class OctreeNode {
   public:
    BoundingBox3f m_box;  // node가 가리키는 box
    std::array<OctreeNode *, 8> m_children = {nullptr};  // node의 children
    bool m_leaf;
    std::vector<uint32_t> m_triangles;
    int m_depth;
    const int MAX_DEPTH=9;
    OctreeNode(BoundingBox3f bbox, std::vector<uint32_t> triangles) {
        m_box = bbox;
        m_triangles = triangles;
        m_depth=0;
        m_leaf= false;
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
                    for (uint32_t idx : m_triangles) {  // 모든 삼각형이 자식 노드의 박스와 만나는지 확인 후 subtriangle에 push
                        if (childBox.overlaps(mesh->getBoundingBox(idx))) {  // 삼각형의 바운딩박스와 박스가 겹쳐지면 만남
                            childTriangles.push_back(idx);
                        }
                    }

                    m_children[int(i * 4 + j * 2 + k)] = new OctreeNode(childBox, childTriangles);
                    m_children[int(i * 4 + j * 2 + k)]->m_depth=this->m_depth+1;
                }
            }
        }
    }

    OctreeNode* build(Mesh *mesh) {
        if (m_triangles.size() == 0) {
            return nullptr;
        }
        if (m_triangles.size() <= 10 || m_depth == MAX_DEPTH) {
            m_leaf=true;
            return this;
        }
        buildChildOctree(mesh);
        for (int i = 0; i < 8; ++i) {
            if(m_children[i]->build(mesh)==nullptr){
                m_leaf=true;
            };
        }
        return this;
    }

    std::vector<uint32_t> findTriangles(Ray3f ray, std::vector<uint32_t> answerTriangles){
        if(m_leaf){return m_triangles;}

        for (int i = 0; i < 8; ++i) {
                if (m_children[i]->m_box.rayIntersect(ray)) {  // childNode가 ray와 만난다면 자식 노드 다시 생성
                    std::vector<uint32_t> result = m_children[i]->findTriangles(ray, answerTriangles);
                    answerTriangles.insert(answerTriangles.end(), result.begin(), result.end());
                }
        }
        return answerTriangles;
    }
};

NORI_NAMESPACE_END

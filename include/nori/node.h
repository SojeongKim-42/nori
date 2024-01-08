#pragma once

#include <nori/accel.h>

NORI_NAMESPACE_BEGIN

class OctreeNode {
   public:
    BoundingBox3f box;  // node가 가리키는 box
    std::array<OctreeNode*, 8> children = {nullptr};  // node의 children
    bool leaf = false;
    OctreeNode(BoundingBox3f bbox) { box = bbox; }

    void buildChildOctree() {
        // 노드의 bounding box를 8개로 나눕니다.
        Vector3f min = box.min;
        Vector3f max = box.max;
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
                    children[int(i * 4 + j * 2 + k)] = new OctreeNode(childBox);
                }
            }
        }
    }
};

NORI_NAMESPACE_END

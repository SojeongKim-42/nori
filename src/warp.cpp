/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/frame.h>
#include <nori/vector.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    // sample은 (0,1) x (0,1), 화면 영역은 (0,1) x (0,1)
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    // sample은 (0,1) x (0,1), 화면 영역은 (0,1) x (0,1)
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f
                                                                        : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
    // p은 (0,1) x (0,1), 화면 영역은 (-1,1) x (-1,1)
    float px = 0, py = 0;

    if (sample[0] >= 0 && sample[0] < 0.5f) {
        px = sqrt(2 * sample[0]) - 1;
    } else if (sample[0] >= 0.5f && sample[0] <= 1) {
        px = 1 - sqrt(2 - 2 * sample[0]);
    }

    if (sample[1] >= 0 && sample[1] < 0.5f) {
        py = sqrt(2 * sample[1]) - 1;
    } else if (sample[1] >= 0.5f && sample[1] <= 1) {
        py = 1 - sqrt(2 - 2 * sample[1]);
    }

    return Point2f(px, py);
}

float Warp::squareToTentPdf(const Point2f &p) {
    // p는 (-1,1) x (-1,1), 화면 영역은 (-1,1) x (-1,1)
    // p가 현재 자리에 있을 확률
    float px = 0, py = 0;

    if (p[0] >= -1 && p[0] <= 1) {
        px = 1 - std::abs(p[0]);
    }

    if (p[1] >= -1 && p[1] <= 1) {
        py = 1 - std::abs(p[1]);
    }

    return px * py;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float r = sqrt(sample[0]);
    float a = sample[1] * 2 * M_PI;
    return Point2f(r * cos(a), r * sin(a));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    if (sqrt((p[0] * p[0] + p[1] * p[1])) >= 1) {
        return 0;
    }
    return 1 / M_PI;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float phi = sample[0] * 2 * M_PI;
    float theta = acos(1 - 2 * sample[1]);
    float x = sin(theta) * cos(phi);
    float y = sin(theta) * sin(phi);
    float z = cos(theta);
    return Vector3f(x, y, z);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    return 1 / (4 * M_PI);
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float phi = sample[0] * 2 * M_PI;
    float theta = acos(1 - sample[1]);
    float x = sin(theta) * cos(phi);
    float y = sin(theta) * sin(phi);
    float z = cos(theta);
    return Vector3f(x, y, z);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    if (v[2] < 0) {
        return 0;
    }
    return 1 / (2 * M_PI);
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    Point2f p = squareToUniformDisk(sample);
    float z = sqrt(std::max((float)0, 1 - p[0] * p[0] - p[1] * p[1]));
    return Vector3f(p[0], p[1], z);
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    if (v[2] < 0) {
        return 0;
    }
    return v[2] / M_PI;  // acos(v[2])=theta
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
}

NORI_NAMESPACE_END

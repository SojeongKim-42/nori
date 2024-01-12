/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/frame.h>
#include <nori/vector.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) { return sample; }

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f
                                                                        : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
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
    throw NoriException("Warp::squareToTentPdf() is not yet implemented!");
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    throw NoriException("Warp::squareToUniformDisk() is not yet implemented!");
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    throw NoriException(
        "Warp::squareToUniformDiskPdf() is not yet implemented!");
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    throw NoriException(
        "Warp::squareToUniformSphere() is not yet implemented!");
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    throw NoriException(
        "Warp::squareToUniformSpherePdf() is not yet implemented!");
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    throw NoriException(
        "Warp::squareToUniformHemisphere() is not yet implemented!");
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    throw NoriException(
        "Warp::squareToUniformHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    throw NoriException(
        "Warp::squareToCosineHemisphere() is not yet implemented!");
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    throw NoriException(
        "Warp::squareToCosineHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
}

NORI_NAMESPACE_END

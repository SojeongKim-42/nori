/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Microfacet : public BSDF {
   public:
    Microfacet(const PropertyList &propList) {
        /* RMS surface roughness */
        m_alpha = propList.getFloat("alpha", 0.1f);

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Albedo of the diffuse base material (a.k.a "kd") */
        m_kd = propList.getColor("kd", Color3f(0.5f));

        /* To ensure energy conservation, we must scale the
           specular component by 1-kd.

           While that is not a particularly realistic model of what
           happens in reality, this will greatly simplify the
           implementation. Please see the course staff if you're
           interested in implementing a more realistic version
           of this BRDF. */
        m_ks = 1 - m_kd.maxCoeff();
    }

    float G(Vector3f wv, Vector3f wh) const {
        Vector3f n = (0, 0, 1);

        if (wv.dot(wh) / wv.dot(n) <= 0) {
            return 0;
        }
        float b = 1 / (m_alpha * Frame::tanTheta(wv));
        if (b < 1.6f) {
            return 3.535f * b +
                   2.181f * b * b / (1 + 2.276f * b + 2.577 * b * b);
        }
        return 1;
    }

    /// Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord &bRec) const {
        Vector3f wh = (bRec.wi + bRec.wo) / pow((bRec.wi + bRec.wo).norm(), 2);
        float cosThetaI = Frame::cosTheta(bRec.wi),
              cosThetaO = Frame::cosTheta(bRec.wo),
              cosThetaH = Frame::cosTheta(wh);

        float D = Warp::squareToBeckmannPdf(wh, m_alpha);
        float F = fresnel(wh.dot(bRec.wi), m_extIOR, m_intIOR);
        float Gi = G(bRec.wi, wh), Go = G(bRec.wo, wh);

        Color3f fr = m_kd / M_PI + m_ks * D * F * Gi * Go /
                                       (4 * cosThetaI * cosThetaO * cosThetaH);
        return fr;
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const {
        Vector3f wh = (bRec.wi + bRec.wo) / pow((bRec.wi + bRec.wo).norm(), 2);
        float Jh = 1 / (4 * wh.dot(bRec.wo));
        float pdf = m_ks * Warp::squareToBeckmannPdf(wh, m_alpha) * Jh +
                    (1 - m_ks) * Frame::cosTheta(bRec.wo) / M_PI;
        return pdf;
    }

    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
        float newSample = _sample[0];
        newSample < 0.5f ? newSample *= 2 : newSample = (newSample - 0.5f) * 2;

        if (newSample < m_ks) { //specular
            Vector3f n = Warp::squareToBeckmann(_sample, m_alpha);
            Frame localFrame(n);
            Vector3f wi = localFrame.toLocal(bRec.wi);
            bRec.wo = localFrame.toWorld(Vector3f(-wi.x(), -wi.y(), wi.z()));
            bRec.measure = EDiscrete;
        } else { //diffuse
            if (Frame::cosTheta(bRec.wi) <= 0) return Color3f(0.0f);
            bRec.measure = ESolidAngle;
            bRec.wo = Warp::squareToCosineHemisphere(_sample);
            bRec.eta = 1.0f;
        }
        return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);

        // Note: Once you have implemented the part that computes the scattered
        // direction, the last part of this function should simply return the
        // BRDF value divided by the solid angle density and multiplied by the
        // cosine factor from the reflection equation, i.e.
        // return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
    }

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    std::string toString() const {
        return tfm::format(
            "Microfacet[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kd = %s,\n"
            "  ks = %f\n"
            "]",
            m_alpha, m_intIOR, m_extIOR, m_kd.toString(), m_ks);
    }

   private:
    float m_alpha;
    float m_intIOR, m_extIOR;
    float m_ks;
    Color3f m_kd;
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END

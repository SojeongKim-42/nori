/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF {
   public:
    Dielectric(const PropertyList &propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
    }

    Color3f eval(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    float pdf(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const {
        float cosThetaI = Frame::cosTheta(bRec.wi);
        Vector3f n = Vector3f(0, 0, 1);
        float etaI = m_extIOR, etaT = m_intIOR;

        if (cosThetaI < 0) {  // outgoing
            cosThetaI = -cosThetaI;
            std::swap(etaI, etaT);
            n = Vector3f(0, 0, -1);
        }

        bRec.eta = etaI / etaT;  // ni/nt
        float sin2ThetaT =
            bRec.eta * bRec.eta * (1 - cosThetaI * cosThetaI);  // snell's law
        float cosThetaT = sqrt(1 - sin2ThetaT);

        // calculate reflection rate -> fresnel equation
        float reflectionRate;
        if (sin2ThetaT > 1.0f)
            reflectionRate = 1.0f;
        else {
            float Rs = (etaI * cosThetaI - etaT * cosThetaT) /
                       (etaI * cosThetaI + etaT * cosThetaT);
            float Rp = (etaT * cosThetaI - etaI * cosThetaT) /
                       (etaT * cosThetaI + etaI * cosThetaT);
            reflectionRate = (Rs * Rs + Rp * Rp) / 2.0f;
        }

        Color3f radiance = 0.f;
        if (sample[0] <= reflectionRate) {  // reflect
            bRec.wo = Vector3f(-bRec.wi.x(), -bRec.wi.y(), bRec.wi.z());
            radiance = reflectionRate;
        } else {  // refract
            bRec.wo =
                bRec.eta * -bRec.wi + (bRec.eta * cosThetaI - cosThetaT) * n;
            if (sin2ThetaT >= 1) return 0.f;  // over critical angle
            radiance = 1 - reflectionRate;
        }
        return radiance;
    }

    std::string toString() const {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }

   private:
    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END

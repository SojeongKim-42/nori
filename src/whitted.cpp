#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <pcg32.h>

NORI_NAMESPACE_BEGIN

class WhittedIntegrator : public Integrator {
   private:
    std::vector<Mesh *> m_emitters;

   public:
    WhittedIntegrator(const PropertyList &props) {}

    void preprocess(const Scene *scene) { m_emitters = scene->getEmitters(); }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        Color3f result = 0;
        if (!scene->rayIntersect(ray, its)) {
            return result;
        }

        if (its.mesh->getBSDF()->isDiffuse() && !its.mesh->isEmitter()) {
            for (Mesh *emitter : m_emitters) {
                Point3f y, x = its.p;  // light sampled point, mesh its point
                Normal3f nY, nX = its.shFrame.n;  // y, x에서의 normal vector
                Point2f random = sampler->next2D();
                float pdfPos = emitter->samplePosition(random, y, nY);
                Vector3f delta = y - x;
                Vector3f wi = delta.normalized();
                // self-intersection을 피하기 위한 offset
                Ray3f shadowRay = Ray3f(x + nX * Epsilon, wi, Epsilon,
                                        delta.norm() - Epsilon);
                if (!scene->rayIntersect(shadowRay)) {
                    BSDFQueryRecord bRec(its.shFrame.toLocal(wi),
                                         its.shFrame.toLocal(-ray.d),
                                         ESolidAngle);
                    Color3f fr = its.mesh->getBSDF()->eval(bRec);
                    Color3f G = abs(nX.dot(wi)) * abs(nY.dot(-wi)) /
                                delta.squaredNorm();
                    Color3f Le = emitter->getEmitter()->getRadiance();
                    Color3f Lr = Le * fr * G;
                    result += Lr / pdfPos;
                }
            }
            result /= m_emitters.size();
        } else {
            if (sampler->next1D() < 0.95f) {
                BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d));
                Color3f weight =
                    its.mesh->getBSDF()->sample(bRec, sampler->next2D());
                if (weight.x() == 0) return 0.f;
                Ray3f newRay = Ray3f(its.p, its.shFrame.toWorld(bRec.wo));
                result += (1 / 0.95) * weight * Li(scene, sampler, newRay);
            }
        }
        return result;
    }

    std::string toString() const { return "WhittedIntegrator[]"; }
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END
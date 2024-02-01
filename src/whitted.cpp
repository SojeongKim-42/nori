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
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its)) return Color3f(0.0f);

        Color3f result = 0;
        Point3f y, x = its.p;  // light 위의 sampled point, intersection point
        Normal3f nY, nX = its.shFrame.n;  // y, x에서의 normal vector
        int N = 10;  // 1로 써도 큰 문제 없음 (sampler sample 수가 크기 때문)

        for (Mesh *emitter : m_emitters) {
            for (int i = 0; i < N; i++) {  // monte carlo integration
                Point2f random = sampler->next2D();
                float pdfPos = emitter->samplePosition(random, y, nY);
                Vector3f delta = y - x;
                Vector3f wi = delta.normalized();

                // self-intersection을 피하기 위한 offset
                Ray3f shadowRay = Ray3f(x + nX * Epsilon, wi, Epsilon,
                                        delta.norm() - Epsilon);

                if (!scene->rayIntersect(shadowRay)) {
                    BSDFQueryRecord bRec(wi, -ray.d, ESolidAngle);
                    Color3f fr = its.mesh->getBSDF()->eval(bRec);

                    Color3f G = abs(nX.dot(wi)) * abs(nY.dot(-wi)) /
                                delta.squaredNorm();

                    Color3f Le = emitter->getEmitter()->getRadiance();

                    Color3f Lr = Le * fr * G;
                    result += Lr / pdfPos;
                }
            }
        }
        result /= N * m_emitters.size();
        return result;
    }

    std::string toString() const { return "WhittedIntegrator[]"; }
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END
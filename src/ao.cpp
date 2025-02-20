#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <pcg32.h>

NORI_NAMESPACE_BEGIN

class AoIntegrator : public Integrator {
   public:
    AoIntegrator(const PropertyList &props) { /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its)) return Color3f(0.0f);

        float result = 0;
        Vector3f x = its.p;
        Vector3f localSample, worldSample;
        int V;
        int N = 1;  // pixel당 sample 수가 줄어들면 N을 늘려야 함

        for (int i = 0; i < N; i++) {
            // sample a hemishpere in local -> transform to world
            localSample = Warp::squareToCosineHemisphere(
                Point2f(sampler->next1D(), sampler->next1D()));
            worldSample = its.shFrame.toWorld(localSample);
            Ray3f shadowRay = Ray3f(x, worldSample);
            scene->rayIntersect(shadowRay) ? V = 0 : V = 1;
            result = result + V;
        }
        result = result / N;
        return Color3f(result);
    }

    std::string toString() const { return "AoIntegrator[]"; }
};

NORI_REGISTER_CLASS(AoIntegrator, "ao");
NORI_NAMESPACE_END

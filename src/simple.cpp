#include <nori/integrator.h>
#include <nori/scene.h>
#include <pcg32.h>

NORI_NAMESPACE_BEGIN

class SimpleIntegrator : public Integrator {
   private:
    Point3f m_position;
    Color3f m_energy;

   public:
    SimpleIntegrator(const PropertyList &props) {
        m_position = props.getPoint("position");
        m_energy = props.getColor("energy");
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its)) return Color3f(0.0f);

        float result;
        Normal3f n = its.shFrame.n;
        Vector3f xTop = m_position - its.p;
        float cosTheta = xTop.dot(n) / (xTop.norm() * n.norm());
        Ray3f shadowRay = Ray3f(its.p, xTop, Epsilon, xTop.norm());
        int V;
        scene->rayIntersect(shadowRay) ? V = 0 : V = 1;

        result = m_energy[0] / (4 * pow(M_PI, 2)) * std::max(0.0f, cosTheta) /
                 pow(xTop.norm(), 2) * V;

        return Color3f(result);
    }

    std::string toString() const { return "SimpleIntegrator[]"; }
};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END